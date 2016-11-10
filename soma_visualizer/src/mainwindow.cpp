#include "mainwindow.h"
#include "ui_mainwindow.h"



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

    this->datetimeformat = "dd-MM-yyyy hh:mm";

    ui->tab->setEnabled(false);

    // We will wait for the map information
    ui->timestepSlider->setEnabled(false);


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

        soma_manager::SOMAQueryObjs query;


        query.request.usedates = true;
        query.request.lowerdate = (this->timelimits.mintimestamp+(value-1)*this->timestep)*1000;
        query.request.upperdate = (this->timelimits.mintimestamp+(value)*this->timestep)*1000;

        this->calculateDateIntervalforTimestep(value);

       /* QDateTime dtlower = this->calculateDateTimeFromTimestamp(query.request.lowerdate);
        QDateTime dtupper = this->calculateDateTimeFromTimestamp(query.request.upperdate);

        QString str = dtlower.toString(this->datetimeformat);

        str +=" - ";
        str+= dtupper.toString(this->datetimeformat);

        ui->datelabel->setText(str);*/

        std::vector<soma_msgs::SOMAObject > somaobjects =  rosthread.querySOMAObjects(query);

        sensor_msgs::PointCloud2 state =  rosthread.getSOMACombinedObjectCloud(somaobjects);

        rosthread.publishSOMAObjectCloud(state);

        ui->noretrievedobjectslabel->setText(QString::number(somaobjects.size()));



        lastqueryjson = QString::fromStdString(query.response.queryjson);

    }
}
QDateTime MainWindow::calculateDateTimeFromTimestamp(long timestamp)
{
    QDateTime dt = QDateTime::fromMSecsSinceEpoch(timestamp,Qt::UTC);

    return dt;

}
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

    this->maxtimestep = round(interval);

    ui->timestepSlider->setMaximum(this->maxtimestep);



    QString labeltext ;
    labeltext.append(QString::number(1));
    labeltext.append(" / ");
    labeltext.append(QString::number(this->maxtimestep));

    ui->timesteplabel->setText(labeltext);

    ui->timestepSlider->setValue(1);
}
void MainWindow::calculateDateIntervalforTimestep(int step)
{

    long lowertimestamp = (this->timelimits.mintimestamp+(step-1)*this->timestep)*1000;
    long uppertimestamp = (this->timelimits.mintimestamp+(step)*this->timestep)*1000;


    QDateTime dtlower = this->calculateDateTimeFromTimestamp(lowertimestamp);
    QDateTime dtupper = this->calculateDateTimeFromTimestamp(uppertimestamp);

    QString str = dtlower.toString(this->datetimeformat);

    str +=" - ";
    str+= dtupper.toString(this->datetimeformat);

    ui->datelabel->setText(str);

}

void MainWindow::handleMapInfoReceived()
{
    ui->tab->setEnabled(true);

    // Enable the slider
    ui->timestepSlider->setEnabled(true);
    ui->sliderCBox->setChecked(true);


    /*************Set Map Name***********************/
    std::string map_name = rosthread.getMapName();

    ui->mapnamelabel->setText(QString::fromStdString(map_name));
    /*************************************************************/


    /***********************Set Timestep Interval *************************************/
    SOMATimeLimits res = this->rosthread.getSOMACollectionMinMaxTimelimits();

    this->timelimits = res;

    qint64 val = res.mintimestamp*1000;
    QDateTime dt = this->calculateDateTimeFromTimestamp(val);
    ui->lowerDateEdit->setDate(dt.date());
    ui->lowerDateEdit->setDisplayFormat("dd-MM-yyyy");

    val = res.maxtimestamp*1000;
    dt = this->calculateDateTimeFromTimestamp(val);
    ui->upperDateEdit->setDate(dt.date());
    ui->upperDateEdit->setDisplayFormat("dd-MM-yyyy");

    ui->lineEditTimeStepIntervalDay->setText("0");
    ui->lineEditTimeStepIntervalDay->setValidator(new QIntValidator(0,30));

    ui->lineEditTimeStepIntervalHours->setText("12");
    ui->lineEditTimeStepIntervalHours->setValidator(new QIntValidator(0,23));

    ui->lineEditTimeStepIntervalMinutes->setText("0");
    ui->lineEditTimeStepIntervalMinutes->setValidator(new QIntValidator(0,59));

    this->calculateSliderLimits(res.mintimestamp,res.maxtimestamp);



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


    soma_manager::SOMAQueryObjs query;

    query.request.usedates = true;
    query.request.lowerdate = (res.mintimestamp+(ui->timestepSlider->value()-1)*this->timestep)*1000;
    query.request.upperdate = (res.mintimestamp+(ui->timestepSlider->value())*this->timestep)*1000;

    this->calculateDateIntervalforTimestep(1);

    std::string queryjson;

    std::vector<soma_msgs::SOMAObject > somaobjects =  rosthread.querySOMAObjects(query);

    sensor_msgs::PointCloud2 state =  rosthread.getSOMACombinedObjectCloud(somaobjects);

    rosthread.publishSOMAObjectCloud(state);

    ui->noretrievedobjectslabel->setText(QString::number(somaobjects.size()));

    this->lastqueryjson = QString::fromStdString(query.response.queryjson);



}
void MainWindow::handleSOMAObjectTypes(std::vector<std::string> typenames)
{
    QString dir = QDir::homePath();

    dir.append("/").append(".soma").append("/objecttypes.txt");

    QFile file(dir);

    if(file.open(QFile::ReadOnly))
    {

        QTextStream stream(&file);


        QStringListModel *model = new QStringListModel(this);

        QStringList list;

        // ui->listViewObjectTypes->setmo

        while(!stream.atEnd())
        {
            QString str = stream.readLine();

            list<<str;

            qDebug()<<str;

            //ui->labelsComboBox->addItem(str);

        }

        model->setStringList(list);

        ui->listViewObjectTypes->setModel(model);
        ui->listViewObjectTypes->setSelectionMode(QAbstractItemView::MultiSelection);
        ui->listViewObjectTypes->setEditTriggers(QAbstractItemView::NoEditTriggers);

        file.close();

    }



    dir = QDir::homePath();

    dir.append("/").append(".soma").append("/objectids.txt");

    QFile file2(dir);

    if(file2.open(QFile::ReadOnly))
    {

        QTextStream stream(&file2);


        QStringListModel *model = new QStringListModel(this);

        QStringList list;

        // ui->listViewObjectTypes->setmo

        while(!stream.atEnd())
        {
            QString str = stream.readLine();

            list<<str;

            qDebug()<<str;

            //ui->labelsComboBox->addItem(str);

        }

        model->setStringList(list);

        ui->listViewObjectIDs->setModel(model);
        ui->listViewObjectIDs->setSelectionMode(QAbstractItemView::MultiSelection);
        ui->listViewObjectIDs->setEditTriggers(QAbstractItemView::NoEditTriggers);


        file2.close();

    }




}
void MainWindow::handleSOMAROINames(std::vector<SOMAROINameIDConfig> roinameidconfigs)
{
    qDebug()<<"ROI Info Received";

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


    if(typeequals || idequals)
    {
        if(typeequals && idequals)
        {
            QModelIndexList indexlist = ui->listViewObjectIDs->selectionModel()->selectedIndexes();

            std::vector<std::string> list;
            std::vector<std::string> typelist;



            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                list.push_back(data.toStdString());
            }




            indexlist = ui->listViewObjectTypes->selectionModel()->selectedIndexes();


            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                typelist.push_back(data.toStdString());
            }




             queryObjects.request.objectids = list;
             queryObjects.request.objecttypes = typelist;

        }
        else if(typeequals){
            QModelIndexList indexlist = ui->listViewObjectTypes->selectionModel()->selectedIndexes();

            std::vector<std::string> list;

            foreach(const QModelIndex& indx, indexlist)
            {
                QString data = indx.data().toString();

                list.push_back(data.toStdString());
            }


            queryObjects.request.objecttypes = list;
        }
        else if(idequals)
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


        queryObjects.request.lowerdate = (this->timelimits.mintimestamp+(ui->timestepSlider->value()-1)*this->timestep)*1000;
        queryObjects.request.upperdate = (this->timelimits.mintimestamp+(ui->timestepSlider->value())*this->timestep)*1000;

        queryObjects.request.usedates = true;

    }

    this->mainBSONObj = mainbuilder.obj();

    mongo::BSONObj tempObject = this->mainBSONObj;

    std::string queryjson;

    std::vector< soma_msgs::SOMAObject > somaobjects =  rosthread.querySOMAObjects(queryObjects);

    ui->noretrievedobjectslabel->setText(QString::number(somaobjects.size()));

    sensor_msgs::PointCloud2 state =  rosthread.getSOMACombinedObjectCloud(somaobjects);

    rosthread.publishSOMAObjectCloud(state);


    lastqueryjson = QString::fromStdString(queryObjects.response.queryjson);



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

    ui->listViewObjectIDs->clearSelection();

    this->lastqueryjson.clear();


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

    qDebug()<<lastindex;

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

}

void MainWindow::on_sliderCBox_clicked(bool checked)
{
    if(checked){

        ui->lowerDateCBox->setChecked(false);
        ui->upperDateCBox->setChecked(false);
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
    ui->timestepSlider->setValue(ui->timestepSlider->maximum());

}

void MainWindow::on_sliderFirstButton_clicked()
{
     ui->timestepSlider->setValue(ui->timestepSlider->minimum());

}
