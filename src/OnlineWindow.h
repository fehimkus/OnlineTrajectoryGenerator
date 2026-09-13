#pragma once

#include <QMainWindow>
#include <QPointF>
#include <QString>

#include <deque>
#include <vector>

#include "TrajectoryGenerator.h"

class QChart;
class QLineSeries;
class QValueAxis;
class QTimer;
class QSlider;
class QDoubleSpinBox;
class QPushButton;
class QLabel;
class QElapsedTimer;

// Test rig for the online trajectory generator.
// A handwheel slider writes TargetPosition, the generator is called once per scan period and the
// four charts show what came out. Nothing about the motion is precomputed - the target is allowed
// to be somewhere else on every single scan.
class OnlineWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit OnlineWindow(QWidget* parent = nullptr);

private slots:
    void onTick();
    void onSliderMoved(int raw);
    void onTargetTyped(double mm);
    void onRandomStep();
    void onCentre();
    void onReset();
    void onPauseToggled();
    void applyParams();

private:
    struct ChartPane
    {
        QChart* chart = nullptr;
        QValueAxis* axX = nullptr;
        QValueAxis* axY = nullptr;
        std::vector<QLineSeries*> traces;           // live time series
        std::vector<std::deque<QPointF>> bufs;      // one ring per trace
        QLineSeries* limHi = nullptr;               // flat limit lines, optional
        QLineSeries* limLo = nullptr;
        double limHiVal = 0.0;
        double limLoVal = 0.0;
        bool hasLim = false;
    };

    ChartPane makePane(const QString& title,
                       const std::vector<QString>& names,
                       const std::vector<QColor>& colors,
                       const std::vector<bool>& dashed,
                       bool withLimits);
    void pushSample(ChartPane& p, int trace, double t, double value);
    void trimPane(ChartPane& p, double tMin);
    void refreshPane(ChartPane& p);

    QWidget* buildControlPanel();
    QWidget* buildChartArea();
    void updateLabels();
    void setTarget(double mm, bool fromSlider);
    void rebuildSliderRange();

    MotionState m_state;
    MotionLimits m_limits;
    TrajectoryStep m_step;
    double m_time = 0.0;                // simulated seconds since the last reset
    double m_dt = 0.001;                // scan period the generator is called with
    double m_targetInput = 0.0;         // what the handwheel says right now, read every scan
    double m_chartAccum = 0.0;          // decimates the chart samples
    double m_chartDt = 0.002;
    bool m_paused = false;
    bool m_syncing = false;             // guards the slider / spin box round trip

    // peaks since the last reset, they prove the limits actually hold
    double m_peakVel = 0.0;
    double m_peakAcc = 0.0;
    double m_peakJerk = 0.0;
    double m_peakError = 0.0;

    QTimer* m_timer = nullptr;
    QElapsedTimer* m_wall = nullptr;

    ChartPane m_posPane, m_velPane, m_accPane, m_jerkPane;

    // handwheel
    QSlider* m_slider = nullptr;
    QDoubleSpinBox* m_spTarget = nullptr;
    QDoubleSpinBox* m_spTravel = nullptr;

    // axis limits
    QDoubleSpinBox* m_spMaxVel = nullptr;
    QDoubleSpinBox* m_spMaxAcc = nullptr;
    QDoubleSpinBox* m_spMaxDec = nullptr;
    QDoubleSpinBox* m_spJerk = nullptr;
    QDoubleSpinBox* m_spNegLimit = nullptr;
    QDoubleSpinBox* m_spPosLimit = nullptr;
    QDoubleSpinBox* m_spWindow = nullptr;

    // rig settings
    QDoubleSpinBox* m_spScan = nullptr;         // ms
    QDoubleSpinBox* m_spSpeed = nullptr;        // real time factor
    QDoubleSpinBox* m_spWindowSec = nullptr;    // chart width in seconds

    QPushButton* m_btnPause = nullptr;

    // read-outs
    QLabel* m_lblPos = nullptr;
    QLabel* m_lblVel = nullptr;
    QLabel* m_lblAcc = nullptr;
    QLabel* m_lblJerk = nullptr;
    QLabel* m_lblVelCmd = nullptr;
    QLabel* m_lblBrake = nullptr;
    QLabel* m_lblStop = nullptr;
    QLabel* m_lblError = nullptr;
    QLabel* m_lblInPos = nullptr;
    QLabel* m_lblPeaks = nullptr;
    QLabel* m_lblScans = nullptr;
};
