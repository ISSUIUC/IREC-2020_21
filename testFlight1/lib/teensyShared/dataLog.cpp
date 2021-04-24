#ifndef DATALOG_CPP
#define DATALOG_CPP

#include <SD.h>

#include "dataLog.h"

#include <stdio.h>

#include <ChRt.h>

#include "sensors.h"



#define THREAD_DEBUG

MUTEX_DECL(SD_Card_Mutex);
/**
 * @brief Construct a new thd function object to log data to the SD card.
 * 
 * @param arg Contains info on the ring buffer containing the data and mutexes protecting it.
 *            This allows the same function to be reused for all data types.
 * 
 */
static THD_FUNCTION(dataLogger_THD, arg){
  struct pointers *pointer_struct = (struct pointers *)arg;
  while(true){
    #ifdef THREAD_DEBUG
        Serial.println("Data Logging thread entrance");
    #endif
    sensorDataStruct_t current_data;

    // Copy low G data
    chSemWait(&pointer_struct->dataloggerTHDVarsPointer->fifoData_lowG);
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex_lowG);
    current_data.lowG_data = pointer_struct->dataloggerTHDVarsPointer->fifoArray[pointer_struct->dataloggerTHDVarsPointer->fifoTail_all].lowG_data;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer->fifoSpace_lowG);
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex_lowG);

    // Copy high G data
    chSemWait(&pointer_struct->dataloggerTHDVarsPointer->fifoData_highG);
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex_highG);
    current_data.highG_data = pointer_struct->dataloggerTHDVarsPointer->fifoArray[pointer_struct->dataloggerTHDVarsPointer->fifoTail_all].highG_data;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer->fifoSpace_highG);
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex_highG);

    // Copy gps data
    chSemWait(&pointer_struct->dataloggerTHDVarsPointer->fifoData_GPS);
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex_GPS);
    current_data.gps_data = pointer_struct->dataloggerTHDVarsPointer->fifoArray[pointer_struct->dataloggerTHDVarsPointer->fifoTail_all].gps_data;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer->fifoSpace_GPS);
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex_GPS);

    // Copy state data
    chSemWait(&pointer_struct->dataloggerTHDVarsPointer->fifoData_state);
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex_state);
    current_data.state_data = pointer_struct->dataloggerTHDVarsPointer->fifoArray[pointer_struct->dataloggerTHDVarsPointer->fifoTail_all].state_data;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer->fifoSpace_state);
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex_state);

    // Copy rocketState data
    chSemWait(&pointer_struct->dataloggerTHDVarsPointer->fifoData_RS);
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex_RS);
    current_data.rocketState_data = pointer_struct->dataloggerTHDVarsPointer->fifoArray[pointer_struct->dataloggerTHDVarsPointer->fifoTail_all].rocketState_data;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer->fifoSpace_RS);
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer->dataMutex_RS);

    // Increment fifoTail after all data has been transfered to current_data
    pointer_struct->dataloggerTHDVarsPointer->fifoTail_all = pointer_struct->dataloggerTHDVarsPointer->fifoTail_all < (FIFO_SIZE - 1) ? pointer_struct->dataloggerTHDVarsPointer->fifoTail_all + 1 : 0;
    
    chMtxLock(&SD_Card_Mutex);
    logData(&pointer_struct->dataloggerTHDVarsPointer->dataFile, &current_data);
    chMtxUnlock(&SD_Card_Mutex);

    // #ifdef THREAD_DEBUG
    //     Serial.println(datalogger_struct->sensor_type);
    //     Serial.println("Data Logging thread exit");
    // #endif

    chThdSleepMilliseconds(6);
  }
}



/**
 * @brief Creates the name for a file to be written to SD card.
 * 
 * @param fileName Pointer to char[] containing intended name of file. Do not include number or file extension at end of name. Make sure this is longer than it needs to be.
 * @param fileExtension Pointer to char[] containing the file extension for the file.
 * @return char* Pointer to inputted char[]. It now contains number (if duplicate file existed) and .csv file extension.
 */
char* sd_file_namer(char* fileName, char* fileExtensionParam) {
    
    char fileExtension[strlen(fileExtensionParam)+1];
    strcpy(fileExtension, fileExtensionParam);
    
    char inputName[strlen(fileName)+1];
    strcpy(inputName,fileName);
    
    strcat(fileName,fileExtension);    

    //checks to see if file already exists and adds 1 to filename if it does.
    if (SD.exists(fileName)){
        bool fileExists = false;
        int i = 1;
        while(fileExists==false){
            if(i > 999){
                //max number of files reached. Don't want to overflow fileName[]. Will write new data to already existing data999.csv
                strcpy(fileName, inputName);
                strcat(fileName, "999");
                strcat(fileName, fileExtension);
                break;
            }

            //converts int i to char[]
            char iStr[16];
            __itoa(i, iStr, 10);

            //writes "(sensor)_data(number).csv to fileNameTemp"
            char fileNameTemp[strlen(inputName)+strlen(iStr)+6];
            strcpy(fileNameTemp, inputName);
            strcat(fileNameTemp,iStr);
            strcat(fileNameTemp, fileExtension);

            if(!SD.exists(fileNameTemp)){
                strcpy(fileName, fileNameTemp);
                fileExists = true;
            }

            i++;
        }
    }

    // Serial.println(fileName);
    return fileName;
}


/**
 * @brief Logs data to 1 line of a specified .csv file on SD card.
 * 
 * @param dataFile File on SD card. Object from SD library.
 * @param data Data structure to be logged.
 * @param sensorType Enum containing the type of sensor. Controls which fields are logged.
 */
int32_t flush_iterator = 0;
void logData(File* dataFile, sensorDataStruct_t* data) {
    // Write raw bytes to SD card.
    dataFile->write((const uint8_t *)data,sizeof(*data));

    // Flush data once for every 1000 writes (this keeps the ring buffer in sync with data collection)
    if (flush_iterator == 1000) {
        dataFile->flush();
        flush_iterator = 0;
    } else {
        flush_iterator++;
    }
    
}

#endif