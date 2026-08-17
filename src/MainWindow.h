#pragma once

#include <QMainWindow>
#include <QPointF>
#include <QString>
#include <deque>

#include "Axis.h"

class QChart;
class QLineSeries;
class QValueAxis;
class QTimer;
class QDoubleSpinBox;
class QPushButton;
class QLabel;
class QCheckBox;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onTick();
    void onRandomize();
    void onLog();
    void applyParams();
    void scheduleNextAutoRandom();

private:
    struct ChartPane
    {
        QChart* chart = nullptr;
        QLineSeries* actual = nullptr;  // actual value
        QLineSeries* ref1 = nullptr;    // reference / limit line
        QLineSeries* ref2 = nullptr;    // second limit line (optional)
        QValueAxis* axX = nullptr;
        QValueAxis* axY = nullptr;
        std::deque<QPointF> buffer;
    };

    ChartPane makePane(const QString& title, const QString& refName,
                       int refCount, const QColor& refColor, double refWidth);
    void pushSample(ChartPane& p, double t, double value);
    void refreshPane(ChartPane& p, double refVal1, double refVal2, bool hasRef2);
    QWidget* buildControlPanel();
    void startBrakeTest();
    void updateLabels();
    static double randIn(double a, double b);

    Axis m_axis;
    double m_time = 0.0;
    double m_dt = 0.001;                // scan period the generator is called with
    double m_chartAccum = 0.0;          // decimates the chart samples

    // --- brake test bookkeeping ---
    bool m_testDone = false;
    double m_startVel = 0.0;            // state the braking started from
    double m_startAcc = 0.0;
    double m_startPos = 0.0;
    double m_startTime = 0.0;
    double m_startDir = 1.0;
    double m_heldBrake = 0.0;           // brake distance frozen at the moment braking started
    double m_measuredDist = 0.0;        // distance really covered until standstill
    double m_measuredTime = 0.0;        // time really needed until standstill
    double m_stopPos = 0.0;             // where it ended up
    double m_lastScanVel = 0.0;         // state going into the scan that hit standstill
    double m_lastScanAcc = 0.0;
    double m_tailTime = 0.0;            // keeps the charts running a bit after standstill
    QString m_branchName;

    QTimer* m_timer = nullptr;
    ChartPane m_posPane, m_velPane, m_accPane;

    // Axis limits that braking actually uses
    QDoubleSpinBox* m_spMaxDec = nullptr;
    QDoubleSpinBox* m_spJerk = nullptr;
    QDoubleSpinBox* m_spScan = nullptr;

    // Random start ranges: [Velocity, Acceleration][min, max]
    QDoubleSpinBox* m_rng[2][2] = {};

    // Automatic random trigger
    QTimer* m_autoTimer = nullptr;
    QCheckBox* m_chkAuto = nullptr;
    QDoubleSpinBox* m_spAutoMin = nullptr; // seconds
    QDoubleSpinBox* m_spAutoMax = nullptr;

    // Brake test read-outs
    QLabel* m_lblBranch = nullptr;
    QLabel* m_lblStart = nullptr;
    QLabel* m_lblHeld = nullptr;
    QLabel* m_lblMeasured = nullptr;
    QLabel* m_lblError = nullptr;
    QLabel* m_lblStopTime = nullptr;
    QLabel* m_lblLiveBrake = nullptr;
    QLabel* m_lblConsistency = nullptr;
    QLabel* m_lblLog = nullptr;
};
