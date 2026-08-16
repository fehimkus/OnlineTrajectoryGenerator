#pragma once

#include <QMainWindow>
#include <QPointF>
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
    void onReset();
    void onStartPause();
    void applyParams();
    void scheduleNextAutoRandom();

private:
    struct ChartPane
    {
        QChart* chart = nullptr;
        QLineSeries* actual = nullptr;  // actual value
        QLineSeries* ref1 = nullptr;    // target / limit line
        QLineSeries* ref2 = nullptr;    // second limit line (optional)
        QValueAxis* axX = nullptr;
        QValueAxis* axY = nullptr;
        std::deque<QPointF> buffer;
    };

    ChartPane makePane(const QString& title, const QString& refName,
                       int refCount, const QColor& refColor, double refWidth);
    void updatePane(ChartPane& p, double newValue,
                    double refVal1, double refVal2, bool hasRef2);
    QWidget* buildControlPanel();
    static double randIn(double a, double b);

    Axis m_axis;
    double m_time = 0.0;

    QTimer* m_timer = nullptr;
    ChartPane m_posPane, m_velPane, m_accPane, m_jerkPane;

    // Target value boxes
    QDoubleSpinBox* m_spMaxVel = nullptr; // target velocity (signed)
    QDoubleSpinBox* m_spMaxAcc = nullptr;
    QDoubleSpinBox* m_spMaxDec = nullptr;
    QDoubleSpinBox* m_spJerk = nullptr;

    // Random range boxes: [Velocity, Acceleration, Deceleration, Jerk][min, max]
    QDoubleSpinBox* m_rng[4][2] = {};

    // Automatic random trigger
    QTimer* m_autoTimer = nullptr;
    QCheckBox* m_chkAuto = nullptr;
    QDoubleSpinBox* m_spAutoMin = nullptr; // seconds
    QDoubleSpinBox* m_spAutoMax = nullptr;

    QPushButton* m_btnStartPause = nullptr;
    QLabel* m_lblState = nullptr;
    QLabel* m_lblPos = nullptr;
    QLabel* m_lblVel = nullptr;
    QLabel* m_lblAcc = nullptr;
    QLabel* m_lblJerk = nullptr;
};
