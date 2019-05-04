#ifndef PROJECT_SPEEDGRAPHPANEL_H
#define PROJECT_SPEEDGRAPHPANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <QtCharts>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QCheckBox>
#include <QSpinBox>
#include <rr_platform/chassis_state.h>
#include <rr_platform/speed.h>
#include <queue>

/*
 * All of our panels need to be under the rr_rviz_plugins namespace.
 */
namespace rr_rviz_plugins {

class SpeedGraphPanel : public rviz::Panel {
Q_OBJECT
public:
    SpeedGraphPanel(QWidget *parent = 0);

protected:
    uint32_t time;
    uint32_t zeroTime;
    bool zeroTimeSet;
    int windowSize;
    int nSamples;
    QSpinBox *nSamplesSpinner;
    double offsetX;
    double offsetY;
    uint32_t axisXMin;
    double axisYMax;
    std::queue<double> averageQueue;
    double currentAverage;
    QLineSeries *currentSeries;
    QLineSeries *averageSeries;
    QLineSeries *goalSpeedSeries;
    QChart *chart;
    QChartView *chartView;
    QCheckBox *autoscrollCheckbox;
    QCheckBox *actualSpeedSeriesCheckbox;
    QCheckBox *averageSpeedSeriesCheckbox;
    QCheckBox *goalSpeedSeriesCheckbox;
    QLabel *averageSpeedLabel;
    QVBoxLayout *layout;
    ros::NodeHandle nh;
    ros::Subscriber chassisSubscriber;
    ros::Subscriber speedSubscriber;
    void chassisCallback(const rr_platform::chassis_state::ConstPtr &msg, QLineSeries *actualSpeedSeries);
    void speedCallback(const rr_platform::speed::ConstPtr &msg);

private slots:
    void actualSpeedSeriesCheckboxCallback(bool);
    void averageSpeedSeriesCheckboxCallback(bool);
    void goalSpeedSeriesCheckboxCallback(bool);
    void autoscrollCallback();
    void nSamplesSpinnerCallback(int);
};

}

#endif
