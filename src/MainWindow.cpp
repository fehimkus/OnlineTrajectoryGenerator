#include "MainWindow.h"
#include "TrajectoryGenerator.h"

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRandomGenerator>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
// Simulation timing
constexpr double SIM_DT = 0.001;      // 1 ms substep
constexpr int SUBSTEPS = 20;          // 20 substeps per 20 ms UI tick
constexpr int TICK_MS = 20;
constexpr double WINDOW_SEC = 10.0;   // sliding time window

// Colors (validated palette)
const QColor COL_SURFACE("#fcfcfb");
const QColor COL_ACTUAL("#2a78d6");   // actual value - blue
const QColor COL_LIMIT("#898781");    // limit - gray, dashed
const QColor COL_GRID("#e1e0d9");
const QColor COL_AXIS_LABEL("#898781");
const QColor COL_TITLE("#52514e");
const QColor COL_BASELINE("#c3c2b7");
} // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("UltimateMotion — Eksen Simülatörü"));

    m_posPane  = makePane(tr("Konum (mm)"),      tr("Limit"), 2, COL_LIMIT, 1.0);
    m_velPane  = makePane(tr("Hız (mm/s)"),      tr("Limit"), 2, COL_LIMIT, 1.0);
    m_accPane  = makePane(tr("İvme (mm/s²)"),    tr("Limit"), 2, COL_LIMIT, 1.0);
    m_jerkPane = makePane(tr("Jerk (mm/s³)"),    tr("Limit"), 2, COL_LIMIT, 1.0);

    auto* chartGrid = new QGridLayout;
    const ChartPane* panes[4] = { &m_posPane, &m_velPane, &m_accPane, &m_jerkPane };
    for (int i = 0; i < 4; ++i)
    {
        auto* view = new QChartView(panes[i]->chart);
        view->setRenderHint(QPainter::Antialiasing);
        chartGrid->addWidget(view, i / 2, i % 2);
    }

    auto* central = new QWidget;
    auto* rootLay = new QHBoxLayout(central);
    rootLay->addWidget(buildControlPanel());
    rootLay->addLayout(chartGrid, /*stretch*/ 1);
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
    panel->setMaximumWidth(340);
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

    // --- Target values ---
    auto* grpTarget = new QGroupBox(tr("Hedef Değerler"));
    auto* form = new QFormLayout(grpTarget);
    m_spMaxVel    = makeSpin(-100000, 100000, m_axis.MaxVelocity, 10, " mm/s");
    m_spMaxAcc    = makeSpin(1, 1000000, m_axis.MaxAcceleration, 50, " mm/s²");
    m_spMaxDec    = makeSpin(1, 1000000, m_axis.MaxDeceleration, 50, " mm/s²");
    m_spJerk      = makeSpin(1, 10000000, m_axis.Jerk, 500, " mm/s³", 0);
    form->addRow(tr("Hedef hız"), m_spMaxVel);
    form->addRow(tr("Maks. ivme"), m_spMaxAcc);
    form->addRow(tr("Maks. yavaşlama"), m_spMaxDec);
    form->addRow(tr("Jerk"), m_spJerk);
    lay->addWidget(grpTarget);

    for (QDoubleSpinBox* sp : { m_spMaxVel, m_spMaxAcc, m_spMaxDec, m_spJerk })
        connect(sp, &QDoubleSpinBox::valueChanged, this, &MainWindow::applyParams);

    // --- Random ranges ---
    auto* grpRng = new QGroupBox(tr("Rastgele Aralıkları"));
    auto* grid = new QGridLayout(grpRng);
    grid->addWidget(new QLabel(tr("Min")), 0, 1);
    grid->addWidget(new QLabel(tr("Maks")), 0, 2);
    const QString rowNames[4] = { tr("Hız"), tr("İvme"),
                                  tr("Yavaşlama"), tr("Jerk") };
    const double defaults[4][2] = {
        { -200.0, 200.0 },   // target velocity (mm/s, signed)
        { 100.0, 2000.0 },   // acceleration (mm/s²)
        { 100.0, 2000.0 },   // deceleration (mm/s²)
        { 500.0, 20000.0 },  // jerk (mm/s³)
    };
    for (int i = 0; i < 4; ++i)
    {
        grid->addWidget(new QLabel(rowNames[i]), i + 1, 0);
        const bool positiveOnly = (i >= 1); // everything except velocity is positive
        const double lo = positiveOnly ? 1.0 : -1000000.0;
        for (int j = 0; j < 2; ++j)
        {
            m_rng[i][j] = makeSpin(lo, 10000000, defaults[i][j], 10, QString(), 0);
            grid->addWidget(m_rng[i][j], i + 1, j + 1);
        }
    }
    auto* btnRandom = new QPushButton(tr("🎲 Rastgele Değer Ata"));
    connect(btnRandom, &QPushButton::clicked, this, &MainWindow::onRandomize);
    grid->addWidget(btnRandom, 5, 0, 1, 3);

    // Automatic trigger: randomizes itself at random intervals
    m_chkAuto = new QCheckBox(tr("🔁 Otomatik tetikle"));
    grid->addWidget(m_chkAuto, 6, 0);
    m_spAutoMin = makeSpin(0.1, 3600, 1.0, 0.5, " s");
    m_spAutoMax = makeSpin(0.1, 3600, 5.0, 0.5, " s");
    grid->addWidget(m_spAutoMin, 6, 1);
    grid->addWidget(m_spAutoMax, 6, 2);
    lay->addWidget(grpRng);

    m_autoTimer = new QTimer(this);
    m_autoTimer->setSingleShot(true);
    connect(m_autoTimer, &QTimer::timeout, this, [this] {
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

    // --- Live values ---
    auto* grpLive = new QGroupBox(tr("Anlık Değerler"));
    auto* liveForm = new QFormLayout(grpLive);
    auto makeValueLabel = [] {
        auto* lbl = new QLabel("0.00");
        QFont f = lbl->font();
        f.setBold(true);
        f.setPointSize(f.pointSize() + 2);
        lbl->setFont(f);
        lbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        return lbl;
    };
    m_lblState = new QLabel(tr("Boşta"));
    m_lblState->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_lblPos = makeValueLabel();
    m_lblVel = makeValueLabel();
    m_lblAcc = makeValueLabel();
    m_lblJerk = makeValueLabel();
    liveForm->addRow(tr("Durum"), m_lblState);
    liveForm->addRow(tr("Konum (mm)"), m_lblPos);
    liveForm->addRow(tr("Hız (mm/s)"), m_lblVel);
    liveForm->addRow(tr("İvme (mm/s²)"), m_lblAcc);
    liveForm->addRow(tr("Jerk (mm/s³)"), m_lblJerk);
    lay->addWidget(grpLive);

    // --- Control buttons ---
    auto* btnLay = new QHBoxLayout;
    m_btnStartPause = new QPushButton(tr("Duraklat"));
    connect(m_btnStartPause, &QPushButton::clicked, this, &MainWindow::onStartPause);
    auto* btnReset = new QPushButton(tr("Sıfırla"));
    connect(btnReset, &QPushButton::clicked, this, &MainWindow::onReset);
    btnLay->addWidget(m_btnStartPause);
    btnLay->addWidget(btnReset);
    lay->addLayout(btnLay);

    lay->addStretch();
    return panel;
}

void MainWindow::onTick()
{
    const double accBefore = m_axis.CurrentAcceleration;
    for (int i = 0; i < SUBSTEPS; ++i)
        GenerateTrajectory(m_axis, SIM_DT);
    m_time += SUBSTEPS * SIM_DT;

    // average jerk applied over the tick
    const double jerkVal =
        (m_axis.CurrentAcceleration - accBefore) / (SUBSTEPS * SIM_DT);

    updatePane(m_posPane, m_axis.CurrentPosition,
               m_axis.PositiveLimit, m_axis.NegativeLimit, true);
    updatePane(m_velPane, m_axis.CurrentVelocity,
               m_axis.MaxVelocity, -m_axis.MaxVelocity, true);
    updatePane(m_accPane, m_axis.CurrentAcceleration,
               m_axis.MaxAcceleration, -m_axis.MaxDeceleration, true);
    updatePane(m_jerkPane, jerkVal,
               m_axis.Jerk, -m_axis.Jerk, true);

    m_lblState->setText(m_axis.CurrentState == AxisState::ContinuousMotion
                            ? tr("Sürüş")
                            : tr("Boşta"));

    m_lblPos->setText(QString::number(m_axis.CurrentPosition, 'f', 2));
    m_lblVel->setText(QString::number(m_axis.CurrentVelocity, 'f', 2));
    m_lblAcc->setText(QString::number(m_axis.CurrentAcceleration, 'f', 2));
    m_lblJerk->setText(QString::number(jerkVal, 'f', 0));
}

void MainWindow::updatePane(ChartPane& p, double newValue,
                            double refVal1, double refVal2, bool hasRef2)
{
    p.buffer.emplace_back(m_time, newValue);
    while (!p.buffer.empty() && p.buffer.front().x() < m_time - WINDOW_SEC)
        p.buffer.pop_front();

    const double x0 = std::max(0.0, m_time - WINDOW_SEC);
    const double x1 = std::max(m_time, WINDOW_SEC);
    p.axX->setRange(x0, x1);

    double lo = std::numeric_limits<double>::max();
    double hi = std::numeric_limits<double>::lowest();
    for (const QPointF& pt : p.buffer)
    {
        lo = std::min(lo, pt.y());
        hi = std::max(hi, pt.y());
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
    const QSignalBlocker b1(m_spMaxVel), b2(m_spMaxAcc), b3(m_spMaxDec),
        b4(m_spJerk);
    m_spMaxVel->setValue(randIn(m_rng[0][0]->value(), m_rng[0][1]->value()));
    m_spMaxAcc->setValue(randIn(m_rng[1][0]->value(), m_rng[1][1]->value()));
    m_spMaxDec->setValue(randIn(m_rng[2][0]->value(), m_rng[2][1]->value()));
    m_spJerk->setValue(randIn(m_rng[3][0]->value(), m_rng[3][1]->value()));

    // start driving to a random target velocity
    applyParams();
    m_axis.CurrentState = AxisState::ContinuousMotion;
}

void MainWindow::scheduleNextAutoRandom()
{
    const double sec = randIn(m_spAutoMin->value(), m_spAutoMax->value());
    m_autoTimer->start(static_cast<int>(sec * 1000.0));
}

void MainWindow::onReset()
{
    m_time = 0.0;
    m_axis.CurrentPosition = 0.0;
    m_axis.CurrentVelocity = 0.0;
    m_axis.CurrentAcceleration = 0.0;
    m_axis.CurrentState = AxisState::Idle;
    for (ChartPane* p : { &m_posPane, &m_velPane, &m_accPane, &m_jerkPane })
        p->buffer.clear();
    applyParams();
}

void MainWindow::onStartPause()
{
    if (m_timer->isActive())
    {
        m_timer->stop();
        m_autoTimer->stop();
        m_btnStartPause->setText(tr("Başlat"));
    }
    else
    {
        m_timer->start();
        if (m_chkAuto->isChecked())
            scheduleNextAutoRandom();
        m_btnStartPause->setText(tr("Duraklat"));
    }
}

void MainWindow::applyParams()
{
    m_axis.MaxVelocity = m_spMaxVel->value();
    m_axis.MaxAcceleration = m_spMaxAcc->value();
    m_axis.MaxDeceleration = m_spMaxDec->value();
    m_axis.Jerk = m_spJerk->value();
}
