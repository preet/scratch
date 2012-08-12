/*
    Copyright (c) 2012 Preet Desai (preet.desai@gmail.com)

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of this software and associated documentation files (the
    "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish,
    distribute, sublicense, and/or sell copies of the Software, and to
    permit persons to whom the Software is furnished to do so, subject to
    the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
    LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
    OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
    WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <sys/time.h>
#include <obdref/parser.h>

// helper function prototypes
void PrintErrors(obdref::Parser &myParser);
void PrintData(QList<obdref::Data> &listData);
void GetSingleFramedResponseFromRandom(obdref::MessageFrame &myMsg);
void GetMultiFramedResponseFromRandom(obdref::MessageFrame &myMsg);
void GetSingleFramedResponseFromTarget(obdref::MessageFrame &myMsg);
void GetMultiFramedResponseFromTarget(obdref::MessageFrame &myMsg);

int main(int argc, char* argv[])
{
    // we expect a single argument that's the path
    // to the definitions file
    bool opOk = false;
    QString filePath(argv[1]);
    if(filePath.isEmpty())
    {
       qDebug() << "Pass the definitions file in as an argument!";   
       return -1;
    }

    // read in xml definitions file and globals js
    obdref::Parser myParser(filePath,opOk);
    if(!opOk)
    {
        qDebug() << "Reading in XML and JS Failed! Exiting...";
        PrintErrors(myParser);
        return -1;
    }
    qDebug() << "OBDREF: Successfully read in XML Defs and JS globals!";

    // get a list of default parameters
    QStringList myParamList;
    myParamList = myParser.GetParameterNames("SAEJ1979",
                                             "ISO 15765-4 Standard",
                                             "Default");

    double timeToBuildMsg = 0;
    double timeToParseMsg = 0;
    for(int i=0; i < myParamList.size(); i++)
    {
        // timing vars
        timeval t1,t2;

        // build a message frame for the current param
        gettimeofday(&t1,NULL);
        obdref::MessageFrame myMsg;
        myMsg.spec = "SAEJ1979";
        myMsg.protocol = "ISO 15765-4 Standard";
        myMsg.address = "Default";
        myMsg.name = myParamList.at(i);

        opOk = myParser.BuildMessageFrame(myMsg);
        if(!opOk)
        {
            qDebug() << "BuildMessageFrame for" << myParamList.at(i)
                     << "Failed! Exiting...";
            PrintErrors(myParser);
            return -1;
        }

        // debug
        qDebug() << "================================================";
        qDebug() << myMsg.spec << "," << myMsg.name;
        qDebug() << myMsg.reqHeaderBytes.data << "|" << myMsg.listMessageData[0].reqDataBytes.data;

        gettimeofday(&t2,NULL);
        timeToBuildMsg += (t2.tv_sec - t1.tv_sec) * 1000.0 * 1000.0;
        timeToBuildMsg += (t2.tv_usec - t1.tv_usec);

        // generate some random data to pretend we
        // have an actual device response
        GetSingleFramedResponseFromRandom(myMsg);

        // parse message frame
        gettimeofday(&t1,NULL);
        QList<obdref::Data> listData;
        opOk = myParser.ParseMessageFrame(myMsg, listData);

        if(!opOk || listData.size() == 0)
        {
            qDebug() << "ParseMessageFrame for" << myParamList.at(i)
                     << "Failed! Exiting...";
            PrintErrors(myParser);
            return -1;
        }
        gettimeofday(&t2,NULL);
        timeToParseMsg += (t2.tv_sec - t1.tv_sec) * 1000.0 * 1000.0;
        timeToParseMsg += (t2.tv_usec - t1.tv_usec);

        // print out data
//        PrintData(listData);
    }

    // print out time taken
    qDebug() << "\n================================================";
    qDebug() << "Stats:";
    qDebug() << "Protocol Used: ISO 15765-4 Standard";
    qDebug() << "Average Time to Build Message:" << timeToBuildMsg/myParamList.size() << "microseconds";
    qDebug() << "Average Time to Parse Message:" << timeToParseMsg/myParamList.size() << "microseconds";
    qDebug() << "\n";

    return 0;
}


// helper functions
void GetSingleFramedResponseFromTarget(obdref::MessageFrame &myMsg)
{
    for(int j=0; j < myMsg.listMessageData.size(); j++)
    {
        obdref::ByteList headerBytes;
        if(myMsg.expHeaderBytes.data.size() > 0)
        {   headerBytes.data.append(myMsg.expHeaderBytes.data);   }
        else
        {
            if(myMsg.protocol == "ISO 15765-4 Standard")
            {   headerBytes.data << 0x07 << 0xE8;   }

            else if(myMsg.protocol == "ISO 15765-4 Extended")
            {   headerBytes.data << 0x18 << 0xDA << 0xF1 << 0x10;    }
        }

        obdref::ByteList dataBytes;
        dataBytes.data.append(0x07);    // single-frame pci byte
        dataBytes.data.append(myMsg.listMessageData[j].expDataPrefix.data);
        int numDataBytes = 8-dataBytes.data.size();
        for(int k=0; k < numDataBytes; k++)
        {
            obdref::ubyte myDataByte = obdref::ubyte(rand() % 256);
            dataBytes.data << myDataByte;
        }

        obdref::ByteList messageBytes;
        messageBytes.data << headerBytes.data << dataBytes.data;
        myMsg.listMessageData[j].listRawDataFrames.append(messageBytes);
    }
}

void GetMultiFramedResponseFromTarget(obdref::MessageFrame &myMsg)
{
    for(int j=0; j < myMsg.listMessageData.size(); j++)
    {
        obdref::ByteList headerBytes;
        if(myMsg.expHeaderBytes.data.size() > 0)
        {   headerBytes.data.append(myMsg.expHeaderBytes.data);   }
        else
        {
            if(myMsg.protocol == "ISO 15765-4 Standard")
            {   headerBytes.data << 0x07 << 0xE8;   }

            else if(myMsg.protocol == "ISO 15765-4 Extended")
            {   headerBytes.data << 0x18 << 0xDA << 0xF1 << 0x10;    }
        }

        obdref::ByteList dataBytes;
        dataBytes.data << 0x15 << 0x05; // first-frame pci bytes (there are 2)
        dataBytes.data.append(myMsg.listMessageData[j].expDataPrefix.data);
        int numDataBytes = 8-dataBytes.data.size();
        for(int k=0; k < numDataBytes; k++)
        {
            obdref::ubyte myDataByte = obdref::ubyte(rand() % 256);
            dataBytes.data << myDataByte;
        }

        obdref::ByteList messageBytes;
        messageBytes.data << headerBytes.data << dataBytes.data;
        myMsg.listMessageData[j].listRawDataFrames.append(messageBytes);

        for(int k=0; k < 4; k++)
        {
            dataBytes.data.clear();
            dataBytes.data << (0x21+k); // consecutive-frame pci byte
            dataBytes.data.append(myMsg.listMessageData[j].expDataPrefix.data);
            for(int k=0; k < 8-dataBytes.data.size(); k++)
            {
                obdref::ubyte myDataByte = obdref::ubyte(rand() % 256);
                dataBytes.data << myDataByte;
            }

            messageBytes.data.clear();
            messageBytes.data << headerBytes.data << dataBytes.data;
            myMsg.listMessageData[j].listRawDataFrames.append(messageBytes);
        }
    }
}

void GetSingleFramedResponseFromRandom(obdref::MessageFrame &myMsg)
{
    for(int j=0; j < myMsg.listMessageData.size(); j++)
    {
        for(int k=0; k < 3; k++)    // responses from three separate addresses
        {
            obdref::ByteList headerBytes;

            if(myMsg.protocol == "ISO 15765-4 Standard")
            {   headerBytes.data << 0x07 << 0xE8+k;   }

            else if(myMsg.protocol == "ISO 15765-4 Extended")
            {   headerBytes.data << 0x18 << 0xDA << 0xF1 << 0x10+k;    }

            obdref::ByteList dataBytes;
            dataBytes.data.append(0x07);    // single-frame
            dataBytes.data.append(myMsg.listMessageData[j].expDataPrefix.data);
            int numDataBytes = 8-dataBytes.data.size();
            for(int l=0; l < numDataBytes; l++)
            {
                obdref::ubyte myDataByte = obdref::ubyte(rand() % 256);
                dataBytes.data << myDataByte;
            }

            obdref::ByteList messageBytes;
            messageBytes.data << headerBytes.data << dataBytes.data;
            myMsg.listMessageData[j].listRawDataFrames.append(messageBytes);
        }
    }
}

void GetMultiFramedResponseFromRandom(obdref::MessageFrame &myMsg)
{
    for(int j=0; j < myMsg.listMessageData.size(); j++)
    {
        for(int k=0; k < 3; k++)    // responses from three separate addresses
        {
            obdref::ByteList headerBytes;

            if(myMsg.protocol == "ISO 15765-4 Standard")
            {   headerBytes.data << 0x07 << 0xE8+k;   }

            else if(myMsg.protocol == "ISO 15765-4 Extended")
            {   headerBytes.data << 0x18 << 0xDA << 0xF1 << 0x10+k;    }

            obdref::ByteList dataBytes;
            dataBytes.data << 0x15 << 0x05; // first-frame pci bytes (there are 2)
            dataBytes.data.append(myMsg.listMessageData[j].expDataPrefix.data);
            int numDataBytes = 8-dataBytes.data.size();
            for(int l=0; l < numDataBytes; l++)
            {
                obdref::ubyte myDataByte = obdref::ubyte(rand() % 256);
                dataBytes.data << myDataByte;
            }

            obdref::ByteList messageBytes;
            messageBytes.data << headerBytes.data << dataBytes.data;
            myMsg.listMessageData[j].listRawDataFrames.append(messageBytes);

            for(int l=0; l < 4; l++)
            {
                dataBytes.data.clear();
                dataBytes.data << (0x21+l); // consecutive-frame pci byte
                dataBytes.data.append(myMsg.listMessageData[j].expDataPrefix.data);
                for(int m=0; m < 8-dataBytes.data.size(); m++)
                {
                    obdref::ubyte myDataByte = obdref::ubyte(rand() % 256);
                    dataBytes.data << myDataByte;
                }

                messageBytes.data.clear();
                messageBytes.data << headerBytes.data << dataBytes.data;
                myMsg.listMessageData[j].listRawDataFrames.append(messageBytes);
            }
        }
    }
}

void PrintErrors(obdref::Parser &myParser)
{
    QStringList listErrors = myParser.GetLastKnownErrors();
    for(int i=0; i < listErrors.size(); i++)
    {   qDebug() << listErrors.at(i);   }
}

void PrintData(QList<obdref::Data> &listData)
{
    for(int i=0; i < listData.size(); i++)
    {
        qDebug() << "================================================";
        qDebug() << listData.at(i).paramName;
        qDebug() << "[From Address]" << listData.at(i).srcAddress;

        if(listData.at(i).listLiteralData.size() > 0)
        {   qDebug() << "[Literal Data]";   }

        for(int j=0; j < listData.at(i).listLiteralData.size(); j++)
        {
            if(listData.at(i).listLiteralData.at(j).value)
            {
                qDebug() << listData.at(i).listLiteralData.at(j).property <<
                "  " << listData.at(i).listLiteralData.at(j).valueIfTrue;
            }
            else
            {
                qDebug() << listData.at(i).listLiteralData.at(j).property <<
                "  " << listData.at(i).listLiteralData.at(j).valueIfFalse;
            }
        }

        if(listData.at(i).listNumericalData.size() > 0)
        {   qDebug() << "[Numerical Data]";   }

        for(int j=0; j < listData.at(i).listNumericalData.size(); j++)
        {
            if(!listData.at(i).listNumericalData.at(j).property.isEmpty())
            {   qDebug() << listData.at(i).listNumericalData.at(j).property;   }

            qDebug() << listData.at(i).listNumericalData.at(j).value
                     << listData.at(i).listNumericalData.at(j).units;
        }
    }
}
