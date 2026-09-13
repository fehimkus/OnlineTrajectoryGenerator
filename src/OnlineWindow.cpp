#include "OnlineWindow.h"
#include "TrajectoryGenerator.h"

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRandomGenerator>
#include <QSlider>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr int TICK_MS = 16;             // gui refresh, one frame
constexpr int SLIDER_TICKS = 200000;    // handwheel resolution, half of it per side
constexpr int MAX_SCANS_PER_TICK = 40000;   // a stalled gui must not turn into a huge time jump
constexpr int MAX_POINTS = 1400;        // per trace, keeps the charts fast

// Colors (validated palette)
const QColor COL_SURFACE("#fcfcfb");
const QColor COL_ACTUAL("#2a78d6");     // actual value - blue
const QColor COL_TARGET("#c8622a");     // target / command - orange
const QColor COL_THIRD("#3f8f6b");      // predicted stop - green
const QColor COL_LIMIT("#898781");      // limit - gray, dashed
const QColor COL_GRID("#e1e0d9");
const QColor COL_AXIS_LABEL("#898781");
const QColor COL_TITLE("#52514e");
const QColor COL_BASELINE("#c3c2b7");

QString num(double v, int prec = 3)
{
    return QString::number(v, 'f', prec);
}
} // namespace

OnlineWindow::OnlineWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("Online Yörünge Üreteci — El Çarkı Testi"));


    auto* split = new QSplitter(Qt::Horizontal, this);
    split->addWidget(buildControlPanel());
    split->addWidget(buildChartArea());
    split->setStretchFactor(0, 0);
    split->setStretchFactor(1, 1);
    setCentralWidget(split);

    applyParams();
    rebuildSliderRange();

    m_wall = new QElapsedTimer;
    m_wall->start();

    m_timer = new QTimer(this);
    m_timer->setInterval(TICK_MS);
    connect(m_timer, &QTimer::timeout, this, &OnlineWindow::onTick);
    m_timer->start();

    resize(1500, 900);
}

// ---------------------------------------------------------------- charts

OnlineWindow::ChartPane OnlineWindow::makePane(const QString& title,
                                               const std::vector<QString>& names,
                                               const std::vector<QColor>& colors,
                                               const std::vector<bool>& dashed,
                                               bool withLimits)
{
    ChartPane p;
    p.chart = new QChart;
    p.chart->setTitle(title);
    p.chart->setTitleBrush(QBrush(COL_TITLE));
    p.chart->setBackgroundBrush(QBrush(COL_SURFACE));
    p.chart->setBackgroundRoundness(0);
    p.chart->setMargins(QMargins(6, 2, 6, 2));
    p.chart->setAnimationOptions(QChart::NoAnimation);

    for (std::size_t i = 0; i < names.size(); ++i)
    {
        auto* s = new QLineSeries;
        s->setName(names[i]);
        s->setUseOpenGL(false);
        QPen pen(colors[i], dashed[i] ? 1.2 : 2.0);
        if (dashed[i])
            pen.setStyle(Qt::DashLine);
        s->setPen(pen);
        p.chart->addSeries(s);
        p.traces.push_back(s);
        p.bufs.emplace_back();
    }

    p.hasLim = withLimits;
    if (withLimits)
    {
        QPen limPen(COL_LIMIT, 1.0, Qt::DotLine);
        p.limHi = new QLineSeries;
        p.limHi->setName(tr("Limit"));
        p.limHi->setPen(limPen);
        p.limLo = new QLineSeries;
        p.limLo->setName(tr("Limit"));
        p.limLo->setPen(limPen);
        p.chart->addSeries(p.limHi);
        p.chart->addSeries(p.limLo);
    }

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

    for (QLineSeries* s : p.traces)
    {
        s->attachAxis(p.axX);
        s->attachAxis(p.axY);
    }
    if (withLimits)
    {
        for (QLineSeries* s : { p.limHi, p.limLo })
        {
            s->attachAxis(p.axX);
            s->attachAxis(p.axY);
        }
        // one legend entry is enough for a symmetric pair
        for (QLegendMarker* m : p.chart->legend()->markers(p.limLo))
            m->setVisible(false);
    }

    p.chart->legend()->setAlignment(Qt::AlignTop);
    p.chart->legend()->setLabelColor(COL_TITLE);
    return p;
}

void OnlineWindow::pushSample(ChartPane& p, int trace, double t, double value)
{
    p.bufs[static_cast<std::size_t>(trace)].emplace_back(t, value);
}

void OnlineWindow::trimPane(ChartPane& p, double tMin)
{
    for (auto& buf : p.bufs)
    {
        while (!buf.empty() && buf.front().x() < tMin)
            buf.pop_front();
        while (buf.size() > MAX_POINTS)
            buf.pop_front();
    }
}

void OnlineWindow::refreshPane(ChartPane& p)
{
    const double width = m_spWindowSec->value();
    const double x1 = std::max(m_time, width);
    const double x0 = x1 - width;
    p.axX->setRange(x0, x1);

    double lo = std::numeric_limits<double>::max();
    double hi = std::numeric_limits<double>::lowest();
    for (const auto& buf : p.bufs)
    {
        for (const QPointF& pt : buf)
        {
            lo = std::min(lo, pt.y());
            hi = std::max(hi, pt.y());
        }
    }
    if (lo > hi)
    {
        lo = 0.0;
        hi = 0.0;
    }
    if (p.hasLim)
    {
        lo = std::min(lo, p.limLoVal);
        hi = std::max(hi, p.limHiVal);
    }

    double span = hi - lo;
    if (span < 1e-9)
        span = std::max(1.0, std::fabs(hi));
    const double pad = span * 0.10;
    p.axY->setRange(lo - pad, hi + pad);

    for (std::size_t i = 0; i < p.traces.size(); ++i)
        p.traces[i]->replace(QList<QPointF>(p.bufs[i].begin(), p.bufs[i].end()));

    if (p.hasLim)
    {
        p.limHi->replace({ QPointF(x0, p.limHiVal), QPointF(x1, p.limHiVal) });
        p.limLo->replace({ QPointF(x0, p.limLoVal), QPointF(x1, p.limLoVal) });
    }
}

QWidget* OnlineWindow::buildChartArea()
{
    m_posPane  = makePane(tr("Konum (mm)"),
                          { tr("Gerçek"), tr("Hedef (el çarkı)"), tr("Tahmini duruş") },
                          { COL_ACTUAL, COL_TARGET, COL_THIRD },
                          { false, true, true }, false);
    m_velPane  = makePane(tr("Hız (mm/s)"),
                          { tr("Gerçek"), tr("Durulma hızı") },
                          { COL_ACTUAL, COL_TARGET },
                          { false, true }, true);
    m_accPane  = makePane(tr("İvme (mm/s²)"),
                          { tr("Gerçek") },
                          { COL_ACTUAL }, { false }, true);
    m_jerkPane = makePane(tr("Jerk (mm/s³)"),
                          { tr("Uygulanan") },
                          { COL_ACTUAL }, { false }, true);

    auto* w = new QWidget;
    auto* grid = new QGridLayout(w);
    grid->setContentsMargins(4, 4, 4, 4);
    grid->setSpacing(4);

    const ChartPane* panes[4] = { &m_posPane, &m_velPane, &m_accPane, &m_jerkPane };
    for (int i = 0; i < 4; ++i)
    {
        auto* view = new QChartView(panes[i]->chart);
        view->setRenderHint(QPainter::Antialiasing);
        view->setMinimumSize(360, 240);
        grid->addWidget(view, i / 2, i % 2);
    }
    return w;
}

// ---------------------------------------------------------------- controls

QWidget* OnlineWindow::buildControlPanel()
{
    auto* panel = new QWidget;
    panel->setMaximumWidth(390);
    auto* col = new QVBoxLayout(panel);

    auto makeSpin = [](double lo, double hi, double val, double step,
                       const QString& suffix, int decimals)
    {
        auto* sp = new QDoubleSpinBox;
        sp->setRange(lo, hi);
        sp->setDecimals(decimals);
        sp->setSingleStep(step);
        sp->setValue(val);
        sp->setSuffix(suffix);
        sp->setKeyboardTracking(false);
        return sp;
    };

    // ---- handwheel ----
    auto* gbWheel = new QGroupBox(tr("El çarkı — hedef konum"));
    auto* wheelBox = new QVBoxLayout(gbWheel);

    m_slider = new QSlider(Qt::Horizontal);
    m_slider->setRange(-SLIDER_TICKS / 2, SLIDER_TICKS / 2);
    m_slider->setValue(0);
    m_slider->setTracking(true);        // has to emit while it is being dragged, not on release
    m_slider->setMinimumHeight(34);
    wheelBox->addWidget(m_slider);

    auto* row = new QHBoxLayout;
    m_spTarget = makeSpin(-100000.0, 100000.0, 0.0, 1.0, tr(" mm"), 4);
    m_spTravel = makeSpin(1.0, 100000.0, 200.0, 10.0, tr(" mm"), 1);
    row->addWidget(new QLabel(tr("Hedef")));
    row->addWidget(m_spTarget, 1);
    row->addWidget(new QLabel(tr("Strok ±")));
    row->addWidget(m_spTravel, 1);
    wheelBox->addLayout(row);

    auto* btnRow = new QHBoxLayout;
    auto* btnCentre = new QPushButton(tr("Merkeze al"));
    auto* btnStep = new QPushButton(tr("Rastgele adım"));
    m_btnPause = new QPushButton(tr("Duraklat"));
    auto* btnReset = new QPushButton(tr("Sıfırla"));
    btnRow->addWidget(btnCentre);
    btnRow->addWidget(btnStep);
    btnRow->addWidget(m_btnPause);
    btnRow->addWidget(btnReset);
    wheelBox->addLayout(btnRow);
    col->addWidget(gbWheel);

    // ---- axis limits ----
    auto* gbLim = new QGroupBox(tr("Eksen limitleri"));
    auto* limForm = new QFormLayout(gbLim);
    m_spMaxVel = makeSpin(0.001, 100000.0, 200.0, 10.0, tr(" mm/s"), 3);
    m_spMaxAcc = makeSpin(0.001, 1000000.0, 1000.0, 50.0, tr(" mm/s²"), 2);
    m_spMaxDec = makeSpin(0.001, 1000000.0, 800.0, 50.0, tr(" mm/s²"), 2);
    m_spJerk   = makeSpin(0.001, 10000000.0, 5000.0, 500.0, tr(" mm/s³"), 1);
    m_spWindow = makeSpin(1e-6, 10.0, 1e-2, 1e-3, tr(" mm"), 6);      // one scan at the residual velocity is already ~3e-4 mm, a smaller window can never be hit
    limForm->addRow(tr("Maks. hız"), m_spMaxVel);
    limForm->addRow(tr("Maks. ivme"), m_spMaxAcc);
    limForm->addRow(tr("Maks. yavaşlama"), m_spMaxDec);
    m_spNegLimit = makeSpin(-100000.0, 100000.0, -1000.0, 10.0, tr(" mm"), 3);
    m_spPosLimit = makeSpin(-100000.0, 100000.0, 1000.0, 10.0, tr(" mm"), 3);
    limForm->addRow(tr("Jerk"), m_spJerk);
    limForm->addRow(tr("Negatif limit"), m_spNegLimit);
    limForm->addRow(tr("Pozitif limit"), m_spPosLimit);
    limForm->addRow(tr("Konum penceresi"), m_spWindow);
    col->addWidget(gbLim);

    // ---- rig ----
    auto* gbRig = new QGroupBox(tr("Test tezgahı"));
    auto* rigForm = new QFormLayout(gbRig);
    m_spScan      = makeSpin(0.001, 100.0, 1.0, 0.05, tr(" ms"), 3);        // deltaTime the generator is called with
    m_spSpeed     = makeSpin(0.02, 1.0, 1.0, 0.05, tr(" ×"), 2);
    m_spWindowSec = makeSpin(0.5, 60.0, 5.0, 0.5, tr(" s"), 1);
    rigForm->addRow(tr("Tarama periyodu"), m_spScan);
    rigForm->addRow(tr("Zaman ölçeği"), m_spSpeed);
    rigForm->addRow(tr("Grafik penceresi"), m_spWindowSec);
    col->addWidget(gbRig);

    // ---- read-outs ----
    auto* gbOut = new QGroupBox(tr("Canlı değerler"));
    auto* outForm = new QFormLayout(gbOut);
    auto addOut = [&outForm](const QString& name) {
        auto* l = new QLabel(QStringLiteral("-"));
        l->setTextInteractionFlags(Qt::TextSelectableByMouse);
        outForm->addRow(name, l);
        return l;
    };
    m_lblPos    = addOut(tr("Konum"));
    m_lblVel    = addOut(tr("Hız"));
    m_lblAcc    = addOut(tr("İvme"));
    m_lblJerk   = addOut(tr("Uygulanan jerk"));
    m_lblVelCmd = addOut(tr("Durulma hızı"));
    m_lblBrake  = addOut(tr("Fren mesafesi"));
    m_lblStop   = addOut(tr("Tahmini duruş"));
    m_lblError  = addOut(tr("Takip hatası"));
    m_lblInPos  = addOut(tr("Konumda"));
    m_lblPeaks  = addOut(tr("Tepe |v| / |a| / |j|"));
    m_lblScans  = addOut(tr("Tarama / kare"));
    col->addWidget(gbOut);

    col->addStretch(1);

    connect(m_slider, &QSlider::valueChanged, this, &OnlineWindow::onSliderMoved);
    connect(m_spTarget, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &OnlineWindow::onTargetTyped);
    connect(m_spTravel, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double) { rebuildSliderRange(); });
    connect(btnCentre, &QPushButton::clicked, this, &OnlineWindow::onCentre);
    connect(btnStep, &QPushButton::clicked, this, &OnlineWindow::onRandomStep);
    connect(btnReset, &QPushButton::clicked, this, &OnlineWindow::onReset);
    connect(m_btnPause, &QPushButton::clicked, this, &OnlineWindow::onPauseToggled);

    for (QDoubleSpinBox* sp : { m_spMaxVel, m_spMaxAcc, m_spMaxDec, m_spJerk, m_spNegLimit, m_spPosLimit,
                                m_spWindow, m_spScan })
    {
        connect(sp, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &OnlineWindow::applyParams);
    }
    return panel;
}

void OnlineWindow::rebuildSliderRange()
{
    const double travel = m_spTravel->value();
    m_spTarget->setRange(-travel, travel);
    setTarget(std::clamp(m_targetInput, -travel, travel), false);
}

void OnlineWindow::setTarget(double mm, bool fromSlider)
{
    m_targetInput = mm;
    m_syncing = true;
    if (!fromSlider)
    {
        const double travel = m_spTravel->value();
        const double frac = (travel > 0.0) ? (mm / travel) : 0.0;
        m_slider->setValue(static_cast<int>(std::lround(frac * (SLIDER_TICKS / 2))));
    }
    if (std::fabs(m_spTarget->value() - mm) > 1e-9)
        m_spTarget->setValue(mm);
    m_syncing = false;
}

void OnlineWindow::onSliderMoved(int raw)
{
    if (m_syncing)
        return;
    const double travel = m_spTravel->value();
    setTarget((static_cast<double>(raw) / (SLIDER_TICKS / 2)) * travel, true);
}

void OnlineWindow::onTargetTyped(double mm)
{
    if (m_syncing)
        return;
    setTarget(mm, false);
}

void OnlineWindow::onCentre()
{
    setTarget(0.0, false);
}

void OnlineWindow::onRandomStep()
{
    auto* rng = QRandomGenerator::global();

    // the limits go in ordered: jerk >= the acceleration limits >= max velocity. the other way round
    // the profile degenerates - the acceleration ramp alone outlasts the move and the axis never
    // takes a proper step, so a random combination that breaks the order tests nothing
    const double vel = 10.0 + (rng->generateDouble() * 490.0);
    const double acc = vel * (1.0 + (rng->generateDouble() * 9.0));
    const double dec = vel * (1.0 + (rng->generateDouble() * 9.0));
    const double jrk = std::max(acc, dec) * (1.0 + (rng->generateDouble() * 9.0));

    for (QDoubleSpinBox* sp : { m_spMaxVel, m_spMaxAcc, m_spMaxDec, m_spJerk })
    {
        sp->blockSignals(true);     // applyParams is called once at the end instead of four times
    }
    m_spMaxVel->setValue(vel);
    m_spMaxAcc->setValue(acc);
    m_spMaxDec->setValue(dec);
    m_spJerk->setValue(jrk);
    for (QDoubleSpinBox* sp : { m_spMaxVel, m_spMaxAcc, m_spMaxDec, m_spJerk })
    {
        sp->blockSignals(false);
    }
    applyParams();

    const double travel = m_spTravel->value();
    setTarget(rng->generateDouble() * 2.0 * travel - travel, false);
}

void OnlineWindow::onPauseToggled()
{
    m_paused = !m_paused;
    m_btnPause->setText(m_paused ? tr("Devam") : tr("Duraklat"));
}

void OnlineWindow::onReset()
{
    m_state.Position = 0.0;
    m_state.Velocity = 0.0;
    m_state.Acceleration = 0.0;
    m_step.BrakeDistance = 0.0;
    m_step.Jerk = 0.0;
    m_time = 0.0;
    m_chartAccum = 0.0;
    m_peakVel = 0.0;
    m_peakAcc = 0.0;
    m_peakJerk = 0.0;
    m_peakError = 0.0;
    for (ChartPane* p : { &m_posPane, &m_velPane, &m_accPane, &m_jerkPane })
    {
        for (auto& buf : p->bufs)
            buf.clear();
    }
    setTarget(0.0, false);
}

void OnlineWindow::applyParams()
{
    m_limits.MaxVelocity = m_spMaxVel->value();
    m_limits.MaxAcceleration = m_spMaxAcc->value();
    m_limits.MaxDeceleration = m_spMaxDec->value();
    m_limits.Jerk = m_spJerk->value();
    m_limits.InPositionWindow = m_spWindow->value();
    m_limits.NegativeLimit = m_spNegLimit->value();
    m_limits.PositiveLimit = m_spPosLimit->value();
    m_limits.InVelocityWindow = 1e-3;       // not on the panel, the rig only drives the position mode
    m_dt = m_spScan->value() / 1000.0;

    // one chart sample every few scans is plenty, the window only holds MAX_POINTS anyway
    m_chartDt = std::max(m_dt, m_spWindowSec->value() / static_cast<double>(MAX_POINTS));

    m_velPane.limHiVal = m_limits.MaxVelocity;
    m_velPane.limLoVal = -m_limits.MaxVelocity;
    m_accPane.limHiVal = m_limits.MaxAcceleration;
    m_accPane.limLoVal = -m_limits.MaxDeceleration;
    m_jerkPane.limHiVal = m_limits.Jerk;
    m_jerkPane.limLoVal = -m_limits.Jerk;
}

// where the velocity settles if the acceleration is nulled from here - the quantity the velocity
// limit is really asked about, and what the rig charts in place of the old velocity command
static double settleVelocity(const MotionState& st, const MotionLimits& lim)
{
    return st.Velocity + ((st.Acceleration * std::fabs(st.Acceleration)) / (2.0 * lim.Jerk));
}

// ---------------------------------------------------------------- the scan loop

void OnlineWindow::onTick()
{
    const double wallSec = static_cast<double>(m_wall->restart()) / 1000.0;

    int scans = 0;
    if (!m_paused)
    {
        // simulated time advances with the wall clock, scaled down if the user wants to watch
        // a transient in slow motion. the cap keeps a stalled frame from turning into a jump
        const double simSec = wallSec * m_spSpeed->value();
        scans = static_cast<int>(simSec / m_dt);
        scans = std::clamp(scans, 0, MAX_SCANS_PER_TICK);
    }

    for (int i = 0; i < scans; ++i)
    {
        // the handwheel is sampled inside the scan loop, exactly like a real encoder read:
        // the generator is given a fresh target on every single scan
        const MotionCommand command = { MOTION_COMMAND_POSITION, m_targetInput };
        m_step = GenerateTrajectory(&m_state, &m_limits, &command, m_dt);
        m_state = m_step.State;

        m_time += m_dt;

        const double err = m_targetInput - m_state.Position;
        m_peakVel = std::max(m_peakVel, std::fabs(m_state.Velocity));
        m_peakAcc = std::max(m_peakAcc, std::fabs(m_state.Acceleration));
        m_peakJerk = std::max(m_peakJerk, std::fabs(m_step.Jerk));
        m_peakError = std::max(m_peakError, std::fabs(err));

        m_chartAccum += m_dt;
        if (m_chartAccum >= m_chartDt)
        {
            m_chartAccum = 0.0;

            // where the axis would come to rest if it started braking on this scan
            const double dirMotion = (std::fabs(m_state.Velocity) > TOLERANCE)
                                         ? ((m_state.Velocity < 0.0) ? -1.0 : 1.0)
                                         : ((m_state.Acceleration < 0.0) ? -1.0 : 1.0);
            const double stopPos = m_state.Position + (m_step.BrakeDistance * dirMotion);

            pushSample(m_posPane, 0, m_time, m_state.Position);
            pushSample(m_posPane, 1, m_time, m_targetInput);
            pushSample(m_posPane, 2, m_time, stopPos);
            pushSample(m_velPane, 0, m_time, m_state.Velocity);
            pushSample(m_velPane, 1, m_time, settleVelocity(m_state, m_limits));
            pushSample(m_accPane, 0, m_time, m_state.Acceleration);
            pushSample(m_jerkPane, 0, m_time, m_step.Jerk);
        }
    }

    const double tMin = m_time - m_spWindowSec->value();
    for (ChartPane* p : { &m_posPane, &m_velPane, &m_accPane, &m_jerkPane })
    {
        trimPane(*p, tMin);
        refreshPane(*p);
    }

    m_lblScans->setText(QString::number(scans));
    updateLabels();
}

void OnlineWindow::updateLabels()
{
    const double err = m_targetInput - m_state.Position;
    const double dirMotion = (std::fabs(m_state.Velocity) > TOLERANCE)
                                 ? ((m_state.Velocity < 0.0) ? -1.0 : 1.0)
                                 : ((m_state.Acceleration < 0.0) ? -1.0 : 1.0);
    const double stopPos = m_state.Position + (m_step.BrakeDistance * dirMotion);

    m_lblPos->setText(num(m_state.Position, 5) + tr(" mm"));
    m_lblVel->setText(num(m_state.Velocity, 3) + tr(" mm/s"));
    m_lblAcc->setText(num(m_state.Acceleration, 2) + tr(" mm/s²"));
    m_lblJerk->setText(num(m_step.Jerk, 1) + tr(" mm/s³"));
    m_lblVelCmd->setText(num(settleVelocity(m_state, m_limits), 3) + tr(" mm/s"));
    m_lblBrake->setText(num(m_step.BrakeDistance, 5) + tr(" mm"));
    m_lblStop->setText(num(stopPos, 5) + tr(" mm"));
    m_lblError->setText(num(err, 5) + tr(" mm"));

    const bool inPos = (std::fabs(err) < m_limits.InPositionWindow) &&
                       (std::fabs(m_state.Velocity) < TOLERANCE);
    m_lblInPos->setText(inPos ? tr("evet") : tr("hayır"));

    m_lblPeaks->setText(num(m_peakVel, 2) + QStringLiteral(" / ") +
                        num(m_peakAcc, 1) + QStringLiteral(" / ") +
                        num(m_peakJerk, 0));
}
