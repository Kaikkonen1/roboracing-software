#include <rr_rviz_plugins/SpeedGraphPanel.h>
#include <QVBoxLayout>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <pluginlib/class_list_macros.h>
#include <rr_platform/chassis_state.h>

namespace rr_rviz_plugins {

SpeedGraphPanel::SpeedGraphPanel(QWidget *parent)
  : rviz::Panel(parent) // Base class constructor
{
    nSamples = 1000;
    currentAverage = 0;
    offsetX = 100;
    offsetY = 0.5;
    axisXMin = 0;
    axisYMax = 0.0;
    time = 0;
    zeroTime = 0;
    zeroTimeSet = false;
    windowSize = 10000; //number of samples to follow

    nSamplesSpinner = new QSpinBox();
    nSamplesSpinner->setPrefix("# samples: ");
    nSamplesSpinner->setValue(nSamples);
    nSamplesSpinner->setMinimum(1);
    nSamplesSpinner->setMaximum(20000);
    connect(nSamplesSpinner, SIGNAL(valueChanged(int)), this, SLOT(nSamplesSpinnerCallback(int)));

    currentSeries = new QLineSeries();
    currentSeries->color() = "blue";
    averageSeries = new QLineSeries();
    averageSeries->color() = "green";
    goalSpeedSeries = new QLineSeries();
    goalSpeedSeries->color() = "red";
    chart = new QChart();
    chart->legend()->hide();
    chart->addSeries(currentSeries);
    chart->addSeries(averageSeries);
    chart->addSeries(goalSpeedSeries);
    chart->createDefaultAxes();
    chart->setTitle("Speed of Vehicle");

    chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    averageSpeedLabel = new QLabel("Avg: 0 m/s");

    autoscrollCheckbox = new QCheckBox("Autoscroll");
    connect(autoscrollCheckbox, SIGNAL (stateChanged(int)), this, SLOT (autoscrollCallback()));


    actualSpeedSeriesCheckbox = new QCheckBox("Actual Speed");
    averageSpeedSeriesCheckbox = new QCheckBox("Average Speed");
    goalSpeedSeriesCheckbox = new QCheckBox("Goal Speed");
    actualSpeedSeriesCheckbox->setChecked(true);
    averageSpeedSeriesCheckbox->setChecked(true);
    goalSpeedSeriesCheckbox->setChecked(true);

    layout = new QVBoxLayout;
    layout->addWidget(chartView);
    layout->addWidget(averageSpeedLabel);
    layout->addWidget(nSamplesSpinner);
    layout->addWidget(autoscrollCheckbox);
    layout->addWidget(actualSpeedSeriesCheckbox);
    layout->addWidget(averageSpeedSeriesCheckbox);
    layout->addWidget(goalSpeedSeriesCheckbox);
    setLayout(layout);

    connect(actualSpeedSeriesCheckbox, SIGNAL (clicked(bool)), this, SLOT (actualSpeedSeriesCheckboxCallback(bool)));
    connect(averageSpeedSeriesCheckbox, SIGNAL (clicked(bool)), this, SLOT (averageSpeedSeriesCheckboxCallback(bool)));
    connect(goalSpeedSeriesCheckbox, SIGNAL (clicked(bool)), this, SLOT (goalSpeedSeriesCheckboxCallback(bool)));

    chassisSubscriber = nh.subscribe<rr_platform::chassis_state>("/chassis_state", 1,
        boost::bind(&SpeedGraphPanel::chassisCallback, this, _1, currentSeries)); //#TODO: no need to boost bind really

    speedSubscriber = nh.subscribe<rr_platform::speed>("/speed", 1, &SpeedGraphPanel::speedCallback, this);

}

void SpeedGraphPanel::chassisCallback(const rr_platform::chassis_stateConstPtr &msg, QLineSeries *actualSpeedSeries) {
    if (!zeroTimeSet) {
        zeroTime = (msg->header.stamp.sec * 1000) + (msg->header.stamp.nsec / 1000000); //milliseconds
        zeroTimeSet = true;
    }

    time = (msg->header.stamp.sec * 1000) + (msg->header.stamp.nsec / 1000000) - zeroTime; //milliseconds

    //curent speed
    if (msg->speed_mps > axisYMax) {
        axisYMax = msg->speed_mps;
        chart->axisY()->setRange(0, axisYMax + offsetY); //auto range speed
    }

    actualSpeedSeries->append(time, msg->speed_mps);

    //average speed
    if (averageQueue.size() < nSamples) {
        averageQueue.push(msg->speed_mps);
        currentAverage = currentAverage + (msg->speed_mps - currentAverage) / averageQueue.size(); //calc incremental mean
    } else {
        //calc rolling mean
        double outValue = averageQueue.front();
        averageQueue.push(msg->speed_mps);
        currentAverage = currentAverage - outValue / nSamples + msg->speed_mps / nSamples;
        averageQueue.pop();
    }

    averageSeries->append(time, currentAverage);
    averageSpeedLabel->setText(("Avg: " + std::to_string(currentAverage) + " m/s").c_str());

    if (autoscrollCheckbox->isChecked()) {
        axisXMin = time - windowSize;
    }

    chart->axisX()->setRange(axisXMin, time + offsetX);

}

void SpeedGraphPanel::speedCallback(const rr_platform::speed::ConstPtr &msg) {
    goalSpeedSeries->append((msg->header.stamp.sec * 1000) + (msg->header.stamp.nsec / 1000000) - zeroTime, msg->speed);
}

void SpeedGraphPanel::actualSpeedSeriesCheckboxCallback(bool checked) {
    if(checked) {
        chart->addSeries(currentSeries);
        chart->createDefaultAxes();
    } else {
        chart->removeSeries(currentSeries);
    }
}
void SpeedGraphPanel::averageSpeedSeriesCheckboxCallback(bool checked) {
    if(checked) {
        chart->addSeries(averageSeries);
        chart->createDefaultAxes();
    } else {
        chart->removeSeries(averageSeries);
    }
}
void SpeedGraphPanel::goalSpeedSeriesCheckboxCallback(bool checked) {
    if(checked) {
        chart->addSeries(goalSpeedSeries);
        chart->createDefaultAxes();
    } else {
        chart->removeSeries(goalSpeedSeries);
    }
}


void SpeedGraphPanel::autoscrollCallback() {
    if (autoscrollCheckbox->isChecked()) {
        axisXMin = time - windowSize;
    } else {
        axisXMin = 0;
    }
}

void SpeedGraphPanel::nSamplesSpinnerCallback(int value) {
    if (value < nSamples) {
        //#TODO: a way to remove one sample and still get average. Shouldn't be hard but I tired
    } else {
        nSamples = value;
        nSamplesSpinner->setMinimum(nSamples);
    }
}

} //rr_rviz_plugins

PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::SpeedGraphPanel, rviz::Panel)
