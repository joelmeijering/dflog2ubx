#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <filesystem>

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_LINUX_NONE
#define CONFIG_HAL_BOARD HAL_BOARD_LINUX
#include "AP_Logger/LogStructure.h"

using namespace std;
using namespace filesystem;

namespace Log{
    static void log(string line, string level){
        cerr << "[" << level << "] " << line << endl;
    }

    static string i(string line){
        log(line, "I");
        return line;
    }

    static string w(string line){
        log(line, "W");
        return line;
    }

    static string e(string line){
        log(line, "!");
        return line;
    }
}

static void writeFile(vector<uint8_t> data, string filename, bool append){
    ios_base::openmode flags = ios::out | ios::binary;
    append && (flags |= ios::app);
    ofstream out(filename, flags);
    out.write(reinterpret_cast<const char*>(data.data()), data.size());
    if(out.fail()){
        throw runtime_error("Error while writing " + to_string(data.size()) + " bytes to " + filename +": " + string(strerror(errno)));
    }
    out.close();
}

int main(int argumentCount, char* arguments[]){
    if(argumentCount < 2){
        throw runtime_error("Provide dflog filename parameter.");
    }
    const string dflogFilename = arguments[1];
    const string ubxFilename = dflogFilename.substr(0, dflogFilename.size() - 4) + ".ubx";
    if(exists(ubxFilename)){
        remove(ubxFilename);
    }

    // UBX_RXM_RAWX: Undividual sattelite measurements, part of UBX_RXM_RAWX struct
    struct UBX_RXM_RAWX_Meas{
        double prMes; // Pseudorange measurement [m]. GLONASS inter frequency channel delays are compensated with an internal calibration table.
        double cpMes; // Carrier phase measurement [cycles]. The carrier phase initial ambiguity is initialized using an approximate value to make the magnitude of the phase close to the pseudorange measurement. Clock resets are applied to both phase and code measurements in accordance with the RINEX speciﬁcation.
        float doMes; // Doppler measurement (positive sign for approaching satellites) [Hz]
        uint8_t gnssId; // GNSS identiﬁer (see Satellite Numbering for a list of identiﬁers)
        uint8_t svId; // Satellite identiﬁer (see Satellite Numbering)
        uint8_t sigId; // New style signal identiﬁer (see Signal Identiﬁers).(not supported for protocol versions less than 27.00)
        uint8_t freqId; // Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
        uint16_t locktime; // Carrier phase locktime counter (maximum 64500ms)
        uint8_t cno; // Carrier-to-noise density ratio (signal strength) [dB-Hz]
        uint8_t prStdev; // Estimated pseudorange measurement standard deviation
        uint8_t cpStdev; // Estimated carrier phase measurement standard deviation (note a raw value of 0x0F indicates the value is invalid)
        uint8_t doStdev; // Estimated Doppler measurement standard deviation.
        uint8_t trkStat; // Tracking status bitﬁeld
        uint8_t reserved1; // Reserved
    };

    // U-Blox RXM-RAWX: Multi-GNSS raw observations message
    struct UBX_RXM_RAWX_Head{
        // Header
        const uint8_t head1 = 0xb5;
        const uint8_t head2 = 0x62;
        const uint8_t cls = 0x02;
        const uint8_t id = 0x15;
        uint16_t len; // Length of body + meas
    };

    struct UBX_RXM_RAWX_Body{
        double rcvTow; // Measurement time of week in receiver local time approximately aligned to the GPS time system. Unit: s 
        uint16_t week;  // GPS week number in receiver local time. Unit: week
        int8_t leapS; // GPS leap seconds (GPS-UTC), best knowledge. Unit s
        uint8_t numMeas; // Number of measurements to follow
        uint8_t recStat; // Receiver tracking status bitfield (bit 0: Leap seconds have been determined, bit 1: Clock reset applied. Typically the receiver clock is changed in increments of integer milliseconds. 
        const uint8_t version = 0x01; // Message version (0x01 for this version)
        uint8_t reserved[2]; // Reserved
    };

    // Sent after each UBX message
    struct UBX_Checksum{
        uint8_t chkA; // 8-bit Fletcher checksum (RFC 1145) calculated over the message, starting and including the class ﬁeld up until, but excluding, the checksum ﬁelds
        uint8_t chkB;
    };

    // Constants
    const size_t READ_AMOUNT = 8192; // APM Planner does this, not sure where they are getting this
    const size_t READ_BUF_SIZE = 8320; // Not sure why this size
    const char HEAD_BYTE1_CHAR = HEAD_BYTE1;
    const char HEAD_BYTE2_CHAR = HEAD_BYTE2;
    const struct LogStructure logStructures[] = {
        LOG_COMMON_STRUCTURES,
        LOG_STRUCTURE_FROM_GPS
    };
    enum ReadState {Head1 = 0, Head2, MsgId, Payload}; // Head1 == Looking for head1 byte, etc.

    // Stats
    size_t statBytesReadTotal = 0; // Total raw input
    int statInvalidMsgId = 0; // Unknown msgIds encountered (msg_type)
    int statIgnMsg = 0;
    int statMsgTotal = 0;
    map<int, int> statMsgCounts; // Amount per msgId
    int statUbxUnexpectedHeader = 0;
    int statUbxUnexpectedMeas = 0;
    int statUbxComplete = 0;

    // Stores the latest chunk read from the file
    vector<char> readBuf;
    readBuf.resize(READ_BUF_SIZE);

    // Build messages in here. Once full message data is received, it is copied out of here 
    char msgData[READ_BUF_SIZE]; // TODO not C++ friendly

    // State
    int state = Head1; 
    uint8_t msgPayloadRemaining; // How many bytes we expect are still coming for current msg
    int msgId; // Id of last msg read
    const struct LogStructure* logStructure; // Located using msgId. Cached for convenience. TODO pointers are not C++ friendly

    // Structures storing output UBX data, it takes multiple messages to fill this struct
    struct UBX_RXM_RAWX{
        UBX_RXM_RAWX_Head head;
        UBX_RXM_RAWX_Body body;
        UBX_RXM_RAWX_Meas meas[255]; // Keep this right below ubxRawxHead unless you want memory corruption 
    } ubxRawx;
    enum UBX_Recv_State {Header = 0, Meas};
    int ubxRecvState = 0;
    uint8_t ubxMeasRemain = 0; // Expecting n more meas (LOG_GPS_RAWS_MSG). Total is found in LOG_GPS_RAWH_MSG 

    // Open file
    ifstream logFile(dflogFilename);
    if(!logFile){
        throw runtime_error("Unable to open file " + dflogFilename);
    }

    // Read chunks from file
    while(true){
        // Read up to READ_AMOUNT into readBuf, set bufBytesRead to amount actually read
        size_t bufBytesRead = READ_AMOUNT;
        logFile.read(readBuf.data(), READ_AMOUNT);
        if(!logFile){
            bufBytesRead = logFile.gcount();
        }
        statBytesReadTotal += bufBytesRead;
        if(bufBytesRead == 0){
            if(logFile.eof()){
                Log::i("Parse complete.");
            }else if(logFile.bad() || logFile.fail()){
                Log::e("Error reading log file.");
            }else{
                Log::w("Read 0 bytes. Reason unknown.");
            }

            Log::i("Total bytes read: " + to_string(statBytesReadTotal));
            Log::i("Total complete msg: " + to_string(statMsgTotal));
            Log::i("Total invalid msg: " + to_string(statInvalidMsgId));      
            Log::i("Total ignored msg: " + to_string(statIgnMsg));
            Log::i("Total ubx unexpected header " + to_string(statUbxUnexpectedHeader));
            Log::i("Total ubx unexpected meas " + to_string(statUbxUnexpectedMeas)); 
            Log::i("Total ubx complete " + to_string(statUbxComplete)); 
            
            for(auto& item: statMsgCounts){
                Log::i("Total msg type " + to_string(item.first) + ": " + to_string(item.second));
            }
            break;
        }

        // Go through the chunk we just read, process each character
        for(int bufOffs = 0; bufOffs < bufBytesRead; bufOffs++){
            switch(state){
                case Head1: 
                case Head2: {
                    if(state == Head1 && readBuf[bufOffs] == HEAD_BYTE1_CHAR ||
                    state == Head2 &&  readBuf[bufOffs] == HEAD_BYTE2_CHAR){
                        state++;
                    }
                    break;
                }
                case MsgId: {
                    msgId = (uint8_t) readBuf[bufOffs];

                    // Find message type to determine structure and length (set LogStructure pointer)
                    logStructure = NULL;
                    for(int i = 0; i < sizeof(logStructures)/sizeof(LogStructure); i++){
                        if(logStructures[i].msg_type == msgId){
                            logStructure = &logStructures[i];
                        }
                    }
                    if(logStructure == NULL){
                        // We have not been able to find matching logStructure for this msgId (msg_type)
                        // Reset state machine, and re-read current byte 
                        state = Head1;
                        bufOffs--;
                        statInvalidMsgId++;
                        continue;
                    }

                    // We found a valid msgId, set expected amount of payload bytes (remove the 3 header bytes, we already have those)
                    msgPayloadRemaining = logStructure->msg_len - 3;
                    
                    state++;

                    break;
                }                    
                case Payload: {
                    // Process message data byte by writing it into the message buffer
                    // Skip first 3 bytes, which are header bytes
                    // We work with msg_len - -3 in this code because we use work with the payload length, which excludes the header 
                    msgData[3 + ((logStructure->msg_len - 3) - msgPayloadRemaining)] = readBuf[bufOffs];
                    msgPayloadRemaining--;

                    if(msgPayloadRemaining == 0){
                        // Message fully consumed
                        statMsgTotal++;
                        statMsgCounts[msgId]++;
                        switch(msgId){
                            case LOG_GPS_MSG: {
                                auto gps = (log_GPS*) msgData;
                                //Log::i("GPS Time: " + to_string(gps->gps_week)); 
                                break;
                            }
                            case LOG_GPS_RAWH_MSG: {
                                auto rawh = (log_GPS_RAWH*) msgData;
                                //Log::i("RAWH WEEK: " + to_string(rawh->week) + " time_us: " + to_string(rawh->time_us));
                                
                                if(ubxRecvState != Header){
                                    // We were expecting a measurement, not a header, take note of it. We restart msg collection.
                                    statUbxUnexpectedHeader++;
                                }

                                ubxRecvState = Meas;
                                ubxMeasRemain = rawh->numMeas;

                                ubxRawx.body.rcvTow = rawh->rcvTow;
                                ubxRawx.body.week = rawh->week;
                                ubxRawx.body.leapS = rawh->leapS;
                                ubxRawx.body.numMeas = rawh->numMeas;
                                //Log::i("numMeas: " + to_string(ubxRawx.body.numMeas));
                                ubxRawx.body.recStat = rawh->recStat;

                                break;
                            }
                            case LOG_GPS_RAWS_MSG: {
                                auto raws = (log_GPS_RAWS*) msgData;
                                //Log::i("RAWS GNSSID: " + to_string(raws->gnssId) + " time_us: " + to_string(raws->time_us));

                                if(ubxRecvState != Meas){
                                    statUbxUnexpectedMeas++;
                                    // We are expecting a header. We cannot assign this measurement to any header so it must be rejected
                                    continue;
                                }

                                // Store the measurement
                                UBX_RXM_RAWX_Meas* meas = &ubxRawx.meas[ubxRawx.body.numMeas - ubxMeasRemain];
                                meas->prMes = raws->prMes;
                                meas->cpMes = raws->cpMes;
                                meas->doMes = raws->doMes;
                                meas->gnssId = raws->gnssId;
                                meas->svId = raws->svId;
                                meas->sigId = raws->sigId;
                                meas->freqId = raws->freqId;
                                meas->locktime = raws->locktime;
                                meas->cno = raws->cno;
                                meas->prStdev = raws->prStdev;
                                meas->cpStdev = raws->cpStdev;
                                meas->doStdev = raws->doStdev;
                                meas->trkStat = raws->trkStat;                                    

                                // We are receiving a measurement, reduce the counter
                                ubxMeasRemain--;

                                // This was the last observation part of the UBX RAWX message. Ready to finalize the UBX message
                                if(ubxMeasRemain == 0){
                                    
                                    statUbxComplete++;
                                    ubxRecvState = Header;

                                    // Calculate length
                                    ubxRawx.head.len = sizeof(UBX_RXM_RAWX_Body) + ubxRawx.body.numMeas * sizeof(UBX_RXM_RAWX_Meas);

                                    vector<uint8_t> outputData;

                                    // Write head
                                    for(int i = 0; i < sizeof(ubxRawx.head); i++){
                                        outputData.push_back(((uint8_t*) &ubxRawx.head)[i]);
                                    }  

                                    // Write body
                                    for(int i = 0; i < sizeof(ubxRawx.body); i++){
                                        outputData.push_back(((uint8_t*) &ubxRawx.body)[i]);
                                    } 

                                    // Write messages
                                    for(int i = 0; i < sizeof(UBX_RXM_RAWX_Meas) * ubxRawx.body.numMeas; i++){
                                        outputData.push_back(((uint8_t*) &ubxRawx.meas)[i]);
                                    } 

                                    // For calculating checksum, we start at the cls field, skipping the 2 header bytes
                                    // Checksum includes everything including the Meas entries
                                    uint8_t startingOffset = (uint8_t) (&ubxRawx.head.cls - &ubxRawx.head.head1);
                                    size_t chksumeeLength = sizeof(UBX_RXM_RAWX_Head) - startingOffset + sizeof(UBX_RXM_RAWX_Body) + sizeof(UBX_RXM_RAWX_Meas) * ubxRawx.body.numMeas;
                                    uint8_t cka = 0, ckb = 0;
                                    for (int i= startingOffset; i < chksumeeLength + startingOffset; i++) {
                                        cka += outputData.data()[i]; ckb += cka;
                                    }

                                    // Write checksum
                                    outputData.push_back(cka);
                                    outputData.push_back(ckb);

                                    writeFile(outputData, ubxFilename, true /*append*/);
                                }

                                break;
                            }
                            default: {
                                statIgnMsg++;
                                break;
                            }
                        }

                        // Reset state machine
                        state = Head1;
                    }
                }
            }
        }
    }
    return 0;
}
