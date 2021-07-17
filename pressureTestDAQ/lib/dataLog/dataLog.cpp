#include <SD.h>

#include "dataLog.h"

void init_dataLog(File* dataFile) {

    char fileName[12];

    strcpy(fileName,"data.csv");

    //checks to see if file already exists and adds 1 to filename if it does.
    if (SD.exists(fileName)){
        bool fileExists = false;
        int i = 1;
        while(fileExists==false){
            if(i > 999){
                //max number of files reached. Don't want to overflow fileName[]. Will write new data to already existing data999.csv
                strcpy(fileName, "data999.csv");
                break;
            }

            //converts int i to char[]
            char iStr[16];
            __itoa(i, iStr, 10);

            //writes "data(number).csv to fileNameTemp"
            char fileNameTemp[10+strlen(iStr)];
            strcpy(fileNameTemp,"data");
            strcat(fileNameTemp,iStr);
            strcat(fileNameTemp,".csv");

            if(!SD.exists(fileNameTemp)){
                strcpy(fileName, fileNameTemp);
                fileExists = true;
            }

            i++;
        }
    }

    Serial.println(fileName);
    *dataFile = SD.open(fileName, O_CREAT | O_WRITE | O_TRUNC);
    dataFile->println("usec, RAW_PT1, RAW_PT2, RAW_PT3, errors");

}


void logData(File* dataFile, FifoItem_t* data) {

    dataFile->print(data->usec);
    dataFile->write(',');
    dataFile->print(data->value1);
    dataFile->write(',');
    dataFile->print(data->value2);
    dataFile->write(',');
    dataFile->print(data->value3);
    dataFile->write(',');
    dataFile->println(data->errors);

    //Writing line of data to SD card
    dataFile->flush();
}
