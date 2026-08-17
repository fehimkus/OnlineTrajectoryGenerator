#include "MainWindow.h"
#include "TrajectoryGenerator.h"

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <QCheckBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRandomGenerator>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
// Simulation timing
constexpr int TICK_MS = 20;
constexpr double TICK_SEC = 0.020;    // simulated seconds per UI tick
constexpr double CHART_DT = 0.0005;   // one chart sample per half millisecond
constexpr double TAIL_SEC = 0.15;     // keep drawing this long after standstill

const QString LOG_FILE = QStringLiteral("braketest_log.csv");

// Colors (validated palette)
const QColor COL_SURFACE("#fcfcfb");
const QColor COL_ACTUAL("#2a78d6");   // actual value - blue
const QColor COL_LIMIT("#898781");    // limit - gray, dashed
const QColor COL_TARGET("#c8622a");   // predicted stop point - orange, dashed
const QColor COL_GRID("#e1e0d9");
const QColor COL_AXIS_LABEL("#898781");
const QColor COL_TITLE("#52514e");
const QColor COL_BASELINE("#c3c2b7");
} // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("Fren Mesafesi Testi"));

    m_posPane = makePane(tr("Konum (mm)"),   tr("Tahmini duruş"), 1, COL_TARGET, 1.5);
    m_velPane = makePane(tr("Hız (mm/s)"),   tr("Sıfır"),         1, COL_LIMIT, 1.0);
    m_accPane = makePane(tr("İvme (mm/s²)"), tr("Yavaşlama limiti"), 2, COL_LIMIT, 1.0);

    auto* chartLay = new QVBoxLayout;
    const ChartPane* panes[3] = { &m_posPane, &m_velPane, &m_accPane };
    for (const ChartPane* p : panes)
    {
        auto* view = new QChartView(p->chart);
        view->setRenderHint(QPainter::Antialiasing);
        chartLay->addWidget(view);
    }

    auto* central = new QWidget;
    auto* rootLay = new QHBoxLayout(central);
    rootLay->addWidget(buildControlPanel());
    rootLay->addLayout(chartLay, /*stretch*/ 1);
    setCentralWidget(central);

    applyParams();

    m_timer = new QTimer(this);
    m_timer->setInterval(TICK_MS);
    connect(m_timer, &QTimer::timeout, this, &MainWindow::onTick);
    m_timer->start();
}

MainWindow::ChartPane MainWindow::makePane(const QString& title, const QString& refName,
                                           int refCount, const QColor& refColor,
                                           double refWidth)
{
    ChartPane p;
    p.chart = new QChart;
    p.chart->setTitle(title);
    p.chart->setTitleBrush(QBrush(COL_TITLE));
    p.chart->setBackgroundBrush(QBrush(COL_SURFACE));
    p.chart->setBackgroundRoundness(0);
    p.chart->setMargins(QMargins(8, 4, 8, 4));
    p.chart->setAnimationOptions(QChart::NoAnimation);

    p.actual = new QLineSeries;
    p.actual->setName(tr("Gerçek"));
    p.actual->setPen(QPen(COL_ACTUAL, 2.0));

    QPen refPen(refColor, refWidth, Qt::DashLine);
    p.ref1 = new QLineSeries;
    p.ref1->setName(refName);
    p.ref1->setPen(refPen);
    if (refCount == 2)
    {
        p.ref2 = new QLineSeries;
        p.ref2->setName(refName);
        p.ref2->setPen(refPen);
    }

    p.chart->addSeries(p.actual);
    p.chart->addSeries(p.ref1);
    if (p.ref2)
        p.chart->addSeries(p.ref2);

    p.axX = new QValueAxis;
    p.axX->setTitleText(tr("t (s)"));
    p.axX->setTickCount(6);
    p.axY = new QValueAxis;
    p.axY->setTickCount(5);
    for (QValueAxis* ax : { p.axX, p.axY })
    {
        ax->setLabelsColor(COL_AXIS_LABEL);
        ax->setTitleBrush(QBrush(COL_AXIS_LABEL));
        ax->setGridLineColor(COL_GRID);
        ax->setLinePen(QPen(COL_BASELINE, 1.0));
    }
    p.chart->addAxis(p.axX, Qt::AlignBottom);
    p.chart->addAxis(p.axY, Qt::AlignLeft);
    for (QLineSeries* s : { p.actual, p.ref1, p.ref2 })
    {
        if (!s)
            continue;
        s->attachAxis(p.axX);
        s->attachAxis(p.axY);
    }

    p.chart->legend()->setAlignment(Qt::AlignTop);
    p.chart->legend()->setLabelColor(COL_TITLE);
    // keep the second reference line with the same name out of the legend
    if (p.ref2)
    {
        const auto markers = p.chart->legend()->markers(p.ref2);
        for (QLegendMarker* m : markers)
            m->setVisible(false);
    }
    return p;
}

QWidget* MainWindow::buildControlPanel()
{
    auto* panel = new QWidget;
    panel->setMaximumWidth(360);
    auto* lay = new QVBoxLayout(panel);

    auto makeSpin = [](double min, double max, double value, double step,
                       const QString& suffix, int decimals = 1) {
        auto* sp = new QDoubleSpinBox;
        sp->setRange(min, max);
        sp->setValue(value);
        sp->setSingleStep(step);
        sp->setSuffix(suffix);
        sp->setDecimals(decimals);
        return sp;
    };

    // --- Limits braking actually uses ---
    auto* grpLimits = new QGroupBox(tr("Limitler"));
    auto* form = new QFormLayout(grpLimits);
    m_spMaxDec = makeSpin(1, 1000000, m_axis.MaxDeceleration, 50, " mm/s²");
    m_spJerk   = makeSpin(1, 10000000, m_axis.Jerk, 500, " mm/s³", 0);
    m_spScan   = makeSpin(0.01, 10.0, 1.0, 0.1, " ms", 2);
    form->addRow(tr("Maks. yavaşlama"), m_spMaxDec);
    form->addRow(tr("Jerk"), m_spJerk);
    form->addRow(tr("Tarama periyodu"), m_spScan);
    lay->addWidget(grpLimits);

    for (QDoubleSpinBox* sp : { m_spMaxDec, m_spJerk, m_spScan })
        connect(sp, &QDoubleSpinBox::valueChanged, this, &MainWindow::applyParams);

    // --- Random start state ---
    auto* grpRng = new QGroupBox(tr("Rastgele Başlangıç Durumu"));
    auto* grid = new QGridLayout(grpRng);
    grid->addWidget(new QLabel(tr("Min")), 0, 1);
    grid->addWidget(new QLabel(tr("Maks")), 0, 2);
    const QString rowNames[2] = { tr("Hız"), tr("İvme") };
    const double defaults[2][2] = {
        { -200.0, 200.0 },     // starting velocity (mm/s, signed)
        { -2000.0, 2000.0 },   // starting acceleration (mm/s², signed)
    };
    for (int i = 0; i < 2; ++i)
    {
        grid->addWidget(new QLabel(rowNames[i]), i + 1, 0);
        for (int j = 0; j < 2; ++j)
        {
            m_rng[i][j] = makeSpin(-1000000, 1000000, defaults[i][j], 10, QString(), 0);
            grid->addWidget(m_rng[i][j], i + 1, j + 1);
        }
    }
    auto* btnRandom = new QPushButton(tr("Rastgele Fren Testi"));
    connect(btnRandom, &QPushButton::clicked, this, &MainWindow::onRandomize);
    grid->addWidget(btnRandom, 3, 0, 1, 3);

    // Automatic trigger: fires a new test once the axis has come to a stop
    m_chkAuto = new QCheckBox(tr("Otomatik tekrarla"));
    grid->addWidget(m_chkAuto, 4, 0);
    m_spAutoMin = makeSpin(0.1, 3600, 1.0, 0.5, " s");
    m_spAutoMax = makeSpin(0.1, 3600, 3.0, 0.5, " s");
    grid->addWidget(m_spAutoMin, 4, 1);
    grid->addWidget(m_spAutoMax, 4, 2);
    lay->addWidget(grpRng);

    m_autoTimer = new QTimer(this);
    m_autoTimer->setSingleShot(true);
    connect(m_autoTimer, &QTimer::timeout, this, [this] {
        if (m_axis.CurrentState == AxisState::Stopping)
        {
            scheduleNextAutoRandom();   // still braking, let it finish first
            return;
        }
        onRandomize();
        scheduleNextAutoRandom();
    });
    connect(m_chkAuto, &QCheckBox::toggled, this, [this](bool on) {
        if (on)
        {
            onRandomize(); // trigger once right away
            scheduleNextAutoRandom();
        }
        else
        {
            m_autoTimer->stop();
        }
    });

    // --- Brake test read-out ---
    auto* grpTest = new QGroupBox(tr("Fren Testi"));
    auto* testForm = new QFormLayout(grpTest);
    auto makeValueLabel = [](bool big) {
        auto* lbl = new QLabel("-");
        QFont f = lbl->font();
        f.setBold(true);
        if (big)
            f.setPointSize(f.pointSize() + 2);
        lbl->setFont(f);
        lbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        return lbl;
    };
    m_lblBranch      = makeValueLabel(false);
    m_lblStart       = makeValueLabel(false);
    m_lblHeld        = makeValueLabel(true);
    m_lblMeasured    = makeValueLabel(true);
    m_lblError       = makeValueLabel(true);
    m_lblStopTime    = makeValueLabel(false);
    m_lblLiveBrake   = makeValueLabel(false);
    m_lblConsistency = makeValueLabel(false);
    testForm->addRow(tr("Seçilen dal"), m_lblBranch);
    testForm->addRow(tr("Başlangıç v / a"), m_lblStart);
    testForm->addRow(tr("Hesaplanan mesafe"), m_lblHeld);
    testForm->addRow(tr("Gerçekleşen mesafe"), m_lblMeasured);
    testForm->addRow(tr("Hata"), m_lblError);
    testForm->addRow(tr("Duruş süresi"), m_lblStopTime);
    testForm->addRow(tr("Canlı fren mesafesi"), m_lblLiveBrake);
    testForm->addRow(tr("Alınan + canlı - tutulan"), m_lblConsistency);
    lay->addWidget(grpTest);

    // --- Log ---
    auto* btnLog = new QPushButton(tr("Log Al"));
    connect(btnLog, &QPushButton::clicked, this, &MainWindow::onLog);
    lay->addWidget(btnLog);

    m_lblLog = new QLabel(tr("henüz log alınmadı"));
    m_lblLog->setWordWrap(true);
    m_lblLog->setStyleSheet("color:#898781;");
    lay->addWidget(m_lblLog);

    lay->addStretch();
    return panel;
}

void MainWindow::startBrakeTest()
{
    // fresh trace for every test
    m_time = 0.0;
    for (ChartPane* p : { &m_posPane, &m_velPane, &m_accPane })
        p->buffer.clear();

    // random state to brake from
    m_axis.CurrentPosition = 0.0;
    m_axis.CurrentVelocity = randIn(m_rng[0][0]->value(), m_rng[0][1]->value());
    m_axis.CurrentAcceleration = randIn(m_rng[1][0]->value(), m_rng[1][1]->value());

    m_startVel = m_axis.CurrentVelocity;
    m_startAcc = m_axis.CurrentAcceleration;
    m_startPos = m_axis.CurrentPosition;
    m_startTime = m_time;
    m_startDir = (m_startVel < 0.0) ? -1.0 : 1.0;

    // which branch of the decision tree this state lands in
    const double product = m_startAcc * m_startVel;
    if (product > TOLERANCE)
        m_branchName = tr("Hızlanıyor (a·v > 0)");
    else if (product < -TOLERANCE)
        m_branchName = tr("Yavaşlıyor (a·v < 0)");
    else
        m_branchName = tr("v veya a sıfır");

    // brake distance is computed every scan, but we freeze the one from the instant
    // braking starts and compare the real motion against it
    GenerateTrajectory(m_axis, m_dt);
    m_heldBrake = m_axis.BrakeDistance;

    m_measuredDist = 0.0;
    m_measuredTime = 0.0;
    m_stopPos = 0.0;
    m_lastScanVel = 0.0;
    m_lastScanAcc = 0.0;
    m_tailTime = 0.0;
    m_chartAccum = 0.0;
    m_testDone = false;

    m_axis.CurrentState = AxisState::Stopping;

    pushSample(m_posPane, m_time, m_axis.CurrentPosition);
    pushSample(m_velPane, m_time, m_axis.CurrentVelocity);
    pushSample(m_accPane, m_time, m_axis.CurrentAcceleration);
}

void MainWindow::onTick()
{
    const int substeps = std::max(1, static_cast<int>(std::lround(TICK_SEC / m_dt)));

    for (int i = 0; i < substeps; ++i)
    {
        const bool braking = (m_axis.CurrentState == AxisState::Stopping);

        if (!braking && m_tailTime <= 0.0)
            break;      // nothing is moving, freeze the trace where it ended

        // state going into this scan, kept for the log if this is the one that stops
        const double scanVel = m_axis.CurrentVelocity;
        const double scanAcc = m_axis.CurrentAcceleration;

        GenerateTrajectory(m_axis, m_dt);       // one scan: brake distance then the motion
        m_time += m_dt;

        bool stoppedNow = false;
        if (braking && m_axis.CurrentState == AxisState::Idle)
        {
            // the axis came to a stop inside this scan
            m_stopPos = m_axis.CurrentPosition;
            m_measuredDist = std::abs(m_axis.CurrentPosition - m_startPos);
            m_measuredTime = m_time - m_startTime;
            m_lastScanVel = scanVel;
            m_lastScanAcc = scanAcc;
            m_tailTime = TAIL_SEC;
            m_testDone = true;
            stoppedNow = true;
        }
        else if (!braking)
        {
            m_tailTime -= m_dt;
        }

        m_chartAccum += m_dt;
        if (m_chartAccum >= CHART_DT || stoppedNow)
        {
            m_chartAccum = 0.0;
            pushSample(m_posPane, m_time, m_axis.CurrentPosition);
            pushSample(m_velPane, m_time, m_axis.CurrentVelocity);
            pushSample(m_accPane, m_time, m_axis.CurrentAcceleration);
        }
    }

    refreshPane(m_posPane, m_startPos + (m_startDir * m_heldBrake), 0.0, false);
    refreshPane(m_velPane, 0.0, 0.0, false);
    refreshPane(m_accPane, m_axis.MaxDeceleration, -m_axis.MaxDeceleration, true);

    updateLabels();
}

void MainWindow::updateLabels()
{
    const bool braking = (m_axis.CurrentState == AxisState::Stopping);

    m_lblBranch->setText(m_branchName.isEmpty() ? QString("-") : m_branchName);
    if (!m_branchName.isEmpty())
    {
        m_lblStart->setText(QString("%1 / %2")
                                .arg(m_startVel, 0, 'f', 2)
                                .arg(m_startAcc, 0, 'f', 0));
        m_lblHeld->setText(QString::number(m_heldBrake, 'f', 4));
    }

    const double travelled = std::abs(m_axis.CurrentPosition - m_startPos);
    const double live = m_axis.BrakeDistance;
    m_lblLiveBrake->setText(braking ? QString::number(live, 'f', 4) : QString("0.0000"));

    // while braking this has to stay on zero: what is left plus what is done equals the held value
    if (!m_branchName.isEmpty())
        m_lblConsistency->setText(QString::number(travelled + live - m_heldBrake, 'f', 4));

    if (m_testDone)
    {
        m_lblMeasured->setText(QString::number(m_measuredDist, 'f', 4));
        m_lblError->setText(QString("%1 mm").arg(m_measuredDist - m_heldBrake, 0, 'f', 4));
        m_lblStopTime->setText(QString("%1 s").arg(m_measuredTime, 0, 'f', 4));
    }
    else if (braking)
    {
        m_lblMeasured->setText(QString::number(travelled, 'f', 4));
        m_lblError->setText(tr("..."));
        m_lblStopTime->setText(QString("%1 s").arg(m_time - m_startTime, 0, 'f', 4));
    }
}

void MainWindow::onLog()
{
    if (!m_testDone)
    {
        m_lblLog->setText(tr("önce bir testin bitmesini bekle"));
        return;
    }

    QFile file(LOG_FILE);
    const bool fresh = !file.exists();

    if (!file.open(QIODevice::Append | QIODevice::Text))
    {
        m_lblLog->setText(tr("log yazılamadı: %1").arg(QFileInfo(file).absoluteFilePath()));
        return;
    }

    QTextStream out(&file);
    if (fresh)
    {
        out << "time,start_vel,start_acc,start_pos,max_dec,jerk,scan_ms,branch,"
               "held_brake,measured_dist,error,stop_time,stop_pos,"
               "vel_before_last_scan,acc_before_last_scan\n";
    }

    auto num = [](double v) { return QString::number(v, 'g', 12); };

    out << QDateTime::currentDateTime().toString(Qt::ISODate) << ','
        << num(m_startVel) << ',' << num(m_startAcc) << ',' << num(m_startPos) << ','
        << num(m_axis.MaxDeceleration) << ',' << num(m_axis.Jerk) << ','
        << num(m_dt * 1000.0) << ',' << '"' << m_branchName << '"' << ','
        << num(m_heldBrake) << ',' << num(m_measuredDist) << ','
        << num(m_measuredDist - m_heldBrake) << ',' << num(m_measuredTime) << ','
        << num(m_stopPos) << ','
        << num(m_lastScanVel) << ',' << num(m_lastScanAcc) << '\n';

    file.close();

    m_lblLog->setText(tr("kaydedildi → %1").arg(QFileInfo(file).absoluteFilePath()));
}

void MainWindow::pushSample(ChartPane& p, double t, double value)
{
    p.buffer.emplace_back(t, value);
}

void MainWindow::refreshPane(ChartPane& p, double refVal1, double refVal2, bool hasRef2)
{
    const double x0 = 0.0;
    const double x1 = std::max(m_time, 0.1);
    p.axX->setRange(x0, x1);

    double lo = std::numeric_limits<double>::max();
    double hi = std::numeric_limits<double>::lowest();
    for (const QPointF& pt : p.buffer)
    {
        lo = std::min(lo, pt.y());
        hi = std::max(hi, pt.y());
    }
    if (p.buffer.empty())
    {
        lo = 0.0;
        hi = 0.0;
    }
    lo = std::min(lo, refVal1);
    hi = std::max(hi, refVal1);
    if (hasRef2)
    {
        lo = std::min(lo, refVal2);
        hi = std::max(hi, refVal2);
    }
    double span = hi - lo;
    if (span < 1e-9)
        span = std::max(1.0, std::fabs(hi));
    const double pad = span * 0.10;
    p.axY->setRange(lo - pad, hi + pad);

    p.actual->replace(QList<QPointF>(p.buffer.begin(), p.buffer.end()));
    p.ref1->replace({ QPointF(x0, refVal1), QPointF(x1, refVal1) });
    if (hasRef2 && p.ref2)
        p.ref2->replace({ QPointF(x0, refVal2), QPointF(x1, refVal2) });
}

double MainWindow::randIn(double a, double b)
{
    if (a > b)
        std::swap(a, b);
    return a + QRandomGenerator::global()->generateDouble() * (b - a);
}

void MainWindow::onRandomize()
{
    applyParams();
    startBrakeTest();
}

void MainWindow::scheduleNextAutoRandom()
{
    const double sec = randIn(m_spAutoMin->value(), m_spAutoMax->value());
    m_autoTimer->start(static_cast<int>(sec * 1000.0));
}

void MainWindow::applyParams()
{
    m_axis.MaxDeceleration = m_spMaxDec->value();
    m_axis.Jerk = m_spJerk->value();
    m_dt = m_spScan->value() / 1000.0;
}
