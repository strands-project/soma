#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "somaobjecttableviewmodel.h"
#include "util.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->setupUI();

    thread = new QThread(this);

    rosthread.moveToThread(thread);

    // Connect the function that will be run when the thread is started
    connect(thread, SIGNAL(started()),&rosthread,SLOT(loop()));

    connect(&rosthread,SIGNAL(mapinfoReceived()),this,SLOT(handleMapInfoReceived()));

    connect(&rosthread,SIGNAL(SOMAROINames(std::vector<SOMAROINameIDConfig>)),this,SLOT(handleSOMAROINames(std::vector<SOMAROINameIDConfig>)));

    connect(&rosthread,SIGNAL(SOMAObjectTypes(std::vector<std::string>)),this,SLOT(handleSOMAObjectTypes(std::vector<std::string>)));

    connect(&rosthread,SIGNAL(rosFinished()),this,SLOT(close()));

    this->thread->start();






    // qDebug()<<map["timestep"];

}
void MainWindow::setupUI()
{
    ui->mapnamelabel->setText("DB Fetch is in progress...");

    this->setWindowTitle("SOMA Visualizer");

    ui->tab->setEnabled(false);

    // We will wait for the map information
    ui->timestepSlider->setEnabled(false);

    ui->tableViewSomaObjects->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    ui->tableViewSomaObjects->setSelectionBehavior(QTableView::SelectRows);

    if(ui->tableViewSomaObjects->model())
        ui->tableViewSomaObjects->model()->deleteLater();

    SomaObjectTableViewModel* mod = new SomaObjectTableViewModel(this);

    ui->tableViewSomaObjects->setModel(mod);



}

MainWindow::~MainWindow()
{
    rosthread.shutdownROS();
    thread->quit();
    thread->wait();
    delete ui;
}


void MainWindow::on_timestepSlider_valueChanged(int value)
{
    // emit sliderValue(value);

    // If slider is Enabled
    if(ui->sliderCBox->isChecked()){

        QString labeltext =  QString::number(value);

        labeltext.append(" / ").append(QString::number(maxtimestep));

        ui->timesteplabel->setText(labeltext);

        //soma_manager::SOMAQueryObjs query;
        this->objectquery.request.usedates = true;
        this->objectquery.request.lowerdate = Util::convertSecTimestamptoMSec(this->timelimits.mintimestamp+(value-1)*this->timestep);
        this->objectquery.request.upperdate = Util::convertSecTimestamptoMSec(this->timelimits.mintimestamp+(value)*this->timestep);

        this->calculateDateIntervalforTimestep(value);


        this->somaobjects =  rosthread.querySOMAObjects(this->objectquery);

        sensor_msgs::PointCloud2 state =  rosthread.getSOMACombinedObjectCloud(somaobjects);

        rosthread.publishSOMAObjectCloud(state);

        ui->noretrievedobjectslabel->setText(QString::number(somaobjects.size()));

        ui->tableViewSomaObjects->model()->deleteLater(); //new SomaObjectTableViewModel(this);

        SomaObjectTableViewModel* newmodel = new SomaObjectTableViewModel(this);

        newmodel->setSOMAObjects(somaobjects);

        ui->tableViewSomaObjects->setModel(newmodel);


        lastqueryjson = QString::fromStdString(this->objectquery.response.queryjson);

    }
}
/*QDateTime MainWindow::calculateDateTimeFromTimestamp(long timestamp)
{
    QDateTime dt = QDateTime::fromMSecsSinceEpoch(timestamp,Qt::UTC);

    return dt;

}*/
void MainWindow::calculateSliderLimits(long lowertimestamp, long uppertimestamp)
{
    long timestepdiff = uppertimestamp-lowertimestamp;




    int daysec = ui->lineEditTimeStepIntervalDay->text().toInt()*24*60*60;
    int hoursec = ui->lineEditTimeStepIntervalHours->text().toInt()*60*60;
    int minsec = ui->lineEditTimeStepIntervalMinutes->text().toInt()*60;



    this->timestep = daysec + hoursec + minsec;

    if(this->timestep == 0)
    {
        ui->lineEditTimeStepIntervalHours->setText("12");
        this->timestep = ui->lineEditTimeStepIntervalHours->text().toInt()*60*60;
    }

    double interval = (double)timestepdiff/this->timestep;

    this->maxtimestep = ceil(interval);

    ui->timestepSlider->setMaximum(this->maxtimestep);
    ui->timestepSlider->setMinimum(1);



    QString labeltext ;
    labeltext.append(QString::number(1));
    labeltext.append(" / ");
    labeltext.append(QString::number(this->maxtimestep));

    ui->timesteplabel->setText(labeltext);

    ui->timestepSlider->setValue(1);
}
void MainWindow::calculateDateIntervalforTimestep(int step)
{

    long lowertimestamp = Util::convertSecTimestamptoMSec(this->timelimits.mintimestamp+(step-1)*this->timestep);
    long uppertimestamp = Util::convertSecTimestamptoMSec(this->timelimits.mintimestamp+(step)*this->timestep);


    QDateTime dtlower = Util::calculateUTCDateTimeFromTimestamp(lowertimestamp);
    QDateTime dtupper = Util::calculateUTCDateTimeFromTimestamp(uppertimestamp);

    QString str = dtlower.toString(datetimeformat);

    str +=" - ";
    str+= dtupper.toString(datetimeformat);

    ui->datelabel->setText(str);

}

void MainWindow::handleMapInfoReceived()
{
    ui->tab->setEnabled(true);


    /*************Set Map Name***********************/
    std::string map_name = rosthread.getMapName();

    ui->mapnamelabel->setText(QString::fromStdString(map_name));
    /*************************************************************/


    /***********************Set Timestep Interval *************************************/
    SOMATimeLimits res = this->rosthread.getSOMACollectionMinMaxTimelimits();

    this->timelimits = res;

    long timeDifference = this->timelimits.maxtimestamp - this->timelimits.mintimestamp;

    // If both limits are equal then there is sth wrong we should return
    if(timeDifference == 0)
    {

        ui->timestepSlider->setEnabled(false);

        ROS_WARN("Time limits of the current object db cannot be determined. Please make sure the database is present...");


        return;
    }


    long interval = timeDifference;

    int min = interval/60;

    if(min > 59) min = 0;

    int hours = interval/(60*60);

    if(hours > 23) hours = 12;

    qint64 val = Util::convertSecTimestamptoMSec(res.mintimestamp);
    QDateTime dt = Util::calculateUTCDateTimeFromTimestamp(val);
    ui->lowerDateEdit->setDate(dt.date());
    ui->lowerDateEdit->setDisplayFormat("dd-MM-yyyy");

    val = Util::convertSecTimestamptoMSec(res.maxtimestamp);
    dt = Util::calculateUTCDateTimeFromTimestamp(val);
    ui->upperDateEdit->setDate(dt.date());
    ui->upperDateEdit->setDisplayFormat("dd-MM-yyyy");

    ui->lineEditTimeStepIntervalDay->setText("0");
    ui->lineEditTimeStepIntervalDay->setValidator(new QIntValidator(0,30));

    ui->lineEditTimeStepIntervalHours->setText(QString::number(hours));
    ui->lineEditTimeStepIntervalHours->setValidator(new QIntValidator(0,23));

    ui->lineEditTimeStepIntervalMinutes->setText(QString::number(min));
    ui->lineEditTimeStepIntervalMinutes->setValidator(new QIntValidator(0,59));

    this->calculateSliderLimits(res.mintimestamp,res.maxtimestamp);

    // Enable the slider
    ui->timestepSlider->setEnabled(true);
    ui->sliderCBox->setChecked(true);




    /********************** Prepare the Weekdays ComboBox ****************************/
    QStringList weekdays;
    weekdays.push_back("");
    weekdays.push_back("Monday");
    weekdays.push_back("Tuesday");
    weekdays.push_back("Wednesday");
    weekdays.push_back("Thursday");
    weekdays.push_back("Friday");
    weekdays.push_back("Saturday");
    weekdays.push_back("Sunday");

    ui->weekdaysComboBox->addItems(weekdays);
    /************************************************************************************/

    // Clear any remaining ROI's in the RVIZ
    rosthread.drawROIwithID("-1");

    ui->timestepSlider->setValue(1);
    emit ui->timestepSlider->valueChanged(1);





}
void MainWindow::handleSOMAObjectTypes(std::vector<std::string> typenames)
{
    QString dir = QDir::homePath();

    dir.append("/").append(".soma").append("/objecttypes.txt");

    QFile file(dir);

    Util::loadListView(&file,ui->listViewObjectTypes);

   /* if(file.open(QFile::ReadOnly))
    {

        QTextStream stream(&file);


        QStringListModel *model = new QStringListModel(this);

        QStringList list;

        // ui->listViewObjectTypes->setmo

        while(!stream.atEnd())
        {
            QString str = stream.readLine();

            list<<str;

            //qDebug()<<str;

            //ui->labelsComboBox->addItem(str);

        }

        model->setStringList(list);

        ui->listViewObjectTypes->setModel(model);
        ui->listViewObjectTypes->setSelectionMode(QAbstractItemView::MultiSelection);
        ui->listViewObjectTypes->setEditTriggers(QAbstractItemView::NoEditTriggers);

        file.close();

    }*/



    dir = QDir::homePath();

    dir.append("/").append(".soma").append("/objectids.txt");

    QFile file2(dir);

    Util::loadListView(&file2,ui->listViewObjectIDs);

    dir = QDir::homePath();

    dir.append("/").append(".soma").append("/objectconfigs.txt");

    QFile file3(dir);

    Util::loadListView(&file3,ui->listViewObjectConfigs);

   /* if(file2.open(QFile::ReadOnly))
    {

        QTextStream stream(&file2);


        QStringListModel *model = new QStringListModel(this);

        QStringList list;

        // ui->listViewObjectTypes->setmo

        while(!stream.atEnd())
        {
            QString str = stream.readLine();

            list<<str;

           // qDebug()<<str;

            //ui->labelsComboBox->addItem(str);

        }

        model->setStringList(list);

        ui->listViewObjectIDs->setModel(model);
        ui->listViewObjectIDs->setSelectionMode(QAbstractItemView::MultiSelection);
        ui->listViewObjectIDs->setEditTriggers(QAbstractItemView::NoEditTriggers);


        file2.close();

    }*/






}
void MainWindow::handleSOMAROINames(std::vector<SOMAROINameIDConfig> roinameidconfigs)
{
    ROS_INFO("ROI Info Received");

    this->roinameidconfigs = roinameidconfigs;

    ui->roiComboBox->addItem("");

    for(int i = 0; i < roinameidconfigs.size();i++)
    {
        QString str;
        str = QString::fromStdString(roinameidconfigs[i].name);

        str.append(" ").append(QString::fromStdString(roinameidconfigs[i].id));

        str.append(" ").append(QString::fromStdString(roinameidconfigs[i].config));

        ui->roiComboBox->addItem(str);

    }


}

void MainWindow::on_roiComboBox_currentIndexChanged(const QString &arg1)
{
    QString str = arg1;
    QStringList numpart = str.split(" ");

    if(numpart.size()>=2){
        // qDebug()<<numpart[1];
        rosthread.drawROIwithID(numpart[1].toStdString());
    }
    else
        rosthread.drawROIwithID("-1");
}

void MainWindow::on_queryButton_clicked()
{
    soma_manager::SOMAQueryObjs queryObjects;

    int roiintindex = ui->roiComboBox->currentIndex();

    int weekdayindex = ui->weekdaysComboBox->currentIndex();

    bool idequals = ui->listViewIDCBox->isChecked();

    bool typeequals = ui->listViewObjectTypesCBox->isChecked();

    bool configequals = ui->listViewConfigCBox->isChecked();

    bool slideractive = ui->sliderCBox->isChecked();

    bool lowertime = ui->lowerTimeCBox->isChecked();

    bool uppertime = ui->upperTimeCBox->isChecked();

    bool lowerdatecbox = ui->lowerDateCBox->isChecked();

    bool upperdatecbox = ui->upperDateCBox->isChecked();

    mongo::BSONObjBuilder mainbuilder;

    if(lowerdatecbox || upperdatecbox)
    {

        queryObjects.request.usedates =true;

        QDateTime datetime;
        datetime.setDate(ui->lowerDateEdit->date());

        QTime time;

        time.setHMS(0,0,0);
        datetime.setTime(time);

        ulong lowerdate =  datetime.toMSecsSinceEpoch();


        datetime.setDate(ui->upperDateEdit->date());
        time.setHMS(23,59,0);
        datetime.setTime(time);

        ulong upperdate = datetime.toMSecsSinceEpoch();


        queryObjects.request.lowerdate = lowerdate;
        queryObjects.request.upperdate = 0;

        if(lowerdatecbox &&  upperdatecbox)
        {


            queryObjects.request.upperdate = upperdate;


        }
        else if(upperdatecbox)
        {
            queryObjects.request.lowerdate = 0;
            queryObjects.request.upperdate = upperdate;
        }





    }

    if(lowertime || uppertime)
    {


        int lowhour = ui->lowerTimeEdit->time().hour();

        int lowmin = ui->lowerTimeEdit->time().minute();

        int upphour = ui->upperTimeEdit->time().hour();

        int uppmin = ui->upperTimeEdit->time().minute();

        queryObjects.request.uselowertime = true;
        queryObjects.request.lowerhour = lowhour;
        queryObjects.request.lowerminutes = lowmin;


        if(lowertime && uppertime)
        {
            queryObjects.request.useuppertime = true;
            queryObjects.request.upperhour = upphour;
            queryObjects.request.upperminutes = uppmin;

        }
        else if(uppertime)
        {
            queryObjects.request.uselowertime = false;
            queryObjects.request.useuppertime = true;
            queryObjects.request.upperhour = upphour;
            queryObjects.request.upperminutes = uppmin;


        }



    }


    if(weekdayindex > 0)
    {
        queryObjects.request.useweekday = true;
        queryObjects.request.weekday = weekdayindex;


    }


    if(typeequals || idequals || configequals)
    {
        if(typeequals)
        {
           /* QModelIndexList indexlist = ui->listViewObjectIDs->selectionModel()->selectedIndexes();

            std::vector<std::string> list;
            std::vector<std::string> typelist;



            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                list.push_back(data.toStdString());
            }*/


            std::vector<std::string> typelist;

            QModelIndexList indexlist = ui->listViewObjectTypes->selectionModel()->selectedIndexes();


            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                typelist.push_back(data.toStdString());
            }



            queryObjects.request.objecttypes = typelist;

        }
       /* else if(typeequals){
            QModelIndexList indexlist = ui->listViewObjectTypes->selectionModel()->selectedIndexes();

            std::vector<std::string> list;

            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                list.push_back(data.toStdString());
            }


            queryObjects.request.objecttypes = list;
        }*/
        if(idequals)
        {
            QModelIndexList indexlist = ui->listViewObjectIDs->selectionModel()->selectedIndexes();

            std::vector<std::string> list;

            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                list.push_back(data.toStdString());
            }



            queryObjects.request.objectids = list;
        }
        if(configequals)
        {
            QModelIndexList indexlist = ui->listViewObjectConfigs->selectionModel()->selectedIndexes();

            std::vector<std::string> list;

            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                list.push_back(data.toStdString());
            }



            queryObjects.request.configs = list;
        }


    }



    if(roiintindex > 0)
    {

        QString roiindex = QString::fromStdString(this->roinameidconfigs[roiintindex-1].id);

        soma_msgs::SOMAROIObject obj =  rosthread.getSOMAROIwithID(roiindex.toInt());


        queryObjects.request.roi_id = obj.id;
        queryObjects.request.useroi_id = true;



    }

    if(slideractive)
    {


        queryObjects.request.lowerdate = Util::convertSecTimestamptoMSec(this->timelimits.mintimestamp+(ui->timestepSlider->value()-1)*this->timestep);
        queryObjects.request.upperdate = Util::convertSecTimestamptoMSec(this->timelimits.mintimestamp+(ui->timestepSlider->value())*this->timestep);

        queryObjects.request.usedates = true;

    }

    this->objectquery = queryObjects;

    this->somaobjects =  rosthread.querySOMAObjects(queryObjects);

    ui->noretrievedobjectslabel->setText(QString::number(somaobjects.size()));

    sensor_msgs::PointCloud2 state =  rosthread.getSOMACombinedObjectCloud(somaobjects);

    rosthread.publishSOMAObjectCloud(state);


    lastqueryjson = QString::fromStdString(queryObjects.response.queryjson);

    ui->tableViewSomaObjects->model()->deleteLater();

    SomaObjectTableViewModel* mod = new SomaObjectTableViewModel(this);
    mod->setSOMAObjects(somaobjects);

    ui->tableViewSomaObjects->setModel(mod);



}


// Reset the Query Fields
void MainWindow::on_resetqueryButton_clicked()
{
    ui->roiComboBox->setCurrentIndex(0);

    ui->weekdaysComboBox->setCurrentIndex(0);

    ui->sliderCBox->setChecked(true);

    ui->listViewObjectTypesCBox->setChecked(false);

    ui->listViewIDCBox->setChecked(false);

    ui->lowerTimeCBox->setChecked(false);

    ui->upperTimeCBox->setChecked(false);

    ui->lowerDateCBox->setChecked(false);

    ui->upperDateCBox->setChecked(false);

    ui->listViewObjectTypes->clearSelection();

    ui->listViewObjectTypes->model()->deleteLater();

    ui->listViewObjectIDs->clearSelection();

    ui->listViewObjectIDs->model()->deleteLater();

    QStringListModel* emptymodel =  new  QStringListModel(this);

    ui->listViewObjectTypes->setModel(emptymodel);

    ui->listViewObjectIDs->setModel(emptymodel);

    this->lastqueryjson.clear();

    this->somaobjects.clear();

    soma_manager::SOMAQueryObjs objs;

    this->objectquery = objs;


    //  emit ui->timestepSlider->valueChanged(mintimestep+1);
    //  ui->timestepSlider->setSliderPosition(mintimestep+1);

    this->setupUI();
    this->rosthread.fetchDataFromDB();





}

void MainWindow::on_exportjsonButton_clicked()
{
    QDialog* dialog = new QDialog(this);

    dialog->setGeometry(100,100,400,400);


    QTextBrowser* browser = new QTextBrowser(dialog);

    browser->setGeometry(0,0,dialog->width(),dialog->height());

    browser->setReadOnly(true);


    /******* Add "new" before Date word for allowing Date queries to be executed directly in RoboMongo ***/

    int index = lastqueryjson.indexOf("Date");

    int lastindex = lastqueryjson.indexOf("Date",index+4);

    //qDebug()<<lastindex;

    if(lastindex > 0)
        lastindex  = lastindex+6;

    if(index > 0 )
    {
        lastqueryjson = lastqueryjson.insert(index-1," new ");

    }
    if(lastindex > 0)
    {
        lastqueryjson = lastqueryjson.insert(lastindex-1," new ");
    }

    /**********************************************************************************************************/

    browser->setText(lastqueryjson);

    dialog->show();

    dialog->setWindowTitle("Query JSON");

    dialog->setAttribute(Qt::WA_DeleteOnClose);


}

void MainWindow::on_sliderCBox_clicked(bool checked)
{
    if(checked)
    {
        ui->timestepSlider->setEnabled(true);

        ui->lowerDateCBox->setChecked(false);
        ui->upperDateCBox->setChecked(false);
    }
    else
    {
        ui->timestepSlider->setEnabled(false);
    }

}



void MainWindow::on_upperDateCBox_clicked(bool checked)
{
    if(checked)
        ui->sliderCBox->setChecked(false);

}

void MainWindow::on_lowerDateCBox_clicked(bool checked)
{
    if(checked)
        ui->sliderCBox->setChecked(false);

}

void MainWindow::on_lineEditTimeStepIntervalMinutes_editingFinished()
{


    this->calculateSliderLimits(this->timelimits.mintimestamp,this->timelimits.maxtimestamp);

    this->calculateDateIntervalforTimestep(1);

}

void MainWindow::on_lineEditTimeStepIntervalHours_editingFinished()
{
    this->calculateSliderLimits(this->timelimits.mintimestamp,this->timelimits.maxtimestamp);

    this->calculateDateIntervalforTimestep(1);


}

void MainWindow::on_lineEditTimeStepIntervalDay_editingFinished()
{
    this->calculateSliderLimits(this->timelimits.mintimestamp,this->timelimits.maxtimestamp);

    this->calculateDateIntervalforTimestep(1);

}

void MainWindow::on_sliderLastButton_clicked()
{
    if(ui->timestepSlider->isEnabled())
        ui->timestepSlider->setValue(ui->timestepSlider->maximum());

}

void MainWindow::on_sliderFirstButton_clicked()
{
    if(ui->timestepSlider->isEnabled())
        ui->timestepSlider->setValue(ui->timestepSlider->minimum());

}

// TODO: Open the item detail viewer for displaying the object details and images
void MainWindow::on_tableViewSomaObjects_doubleClicked(const QModelIndex &index)
{
    SomaObjectDetailDialog* dialog =  new SomaObjectDetailDialog(this,somaobjects[index.row()]);

    dialog->show();

    dialog->setAttribute(Qt::WA_DeleteOnClose);


}
