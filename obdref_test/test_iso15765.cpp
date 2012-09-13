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

void GetSFResponseFromTarget(obdref::MessageFrame &myMsg);
void GetSFResponseFromRandom(obdref::MessageFrame &myMsg);
void GetMFResponseFromTarget(obdref::MessageFrame &myMsg);
void GetMFResponseFromRandom(obdref::MessageFrame &myMsg);

void PrintErrors(obdref::Parser &myParser);
void PrintData(QList<obdref::Data> &listData);
void PrintReqResp(const obdref::MessageFrame &msgFrame);

int main(int argc, char* argv[])
{
    // we expect a single argument that's the path
    // to the definitions file
    bool opOk = false;
    QString filePath(argv[1]);
    if(filePath.isEmpty())   {
       qDebug() << "Pass the definitions file in as an argument!";   
       return -1;
    }

    // read in xml definitions file and globals js
    obdref::Parser myParser(filePath,opOk);
    if(!opOk)   {
        qDebug() << "Reading in XML and JS Failed! Exiting...";
        PrintErrors(myParser);
        return -1;
    }
    qDebug() << "OBDREF: Successfully read in XML Defs and JS globals!";

    // get a list of default parameters
    QStringList myParamList
        = myParser.GetParameterNames("SAEJ1979",
                                     "ISO 15765-4 Standard",
                                     "Default");

    for(size_t i=0; i < myParamList.size(); i++)
    {
        QList<obdref::Data> listData;

        // create message frame
        obdref::MessageFrame myMsg;
        myMsg.spec = "SAEJ1979";
        myMsg.protocol = "ISO 15765-4 Standard";
        myMsg.address = "Default";
        myMsg.name = myParamList.at(i);

        // build a message frame
        opOk = myParser.BuildMessageFrame(myMsg);
        if(!opOk)   {
            PrintErrors(myParser);
            return -1;
        }

        // simulate vehicle response
        GetMFResponseFromRandom(myMsg);
        GetSFResponseFromRandom(myMsg);
        GetMFResponseFromTarget(myMsg);
//        PrintReqResp(myMsg);

        // parse message frame
        opOk = myParser.ParseMessageFrame(myMsg,listData);
        if(!opOk)   {
            PrintErrors(myParser);
            return -1;
        }

        // print out data
        PrintData(listData);
    }

    return 0;
}

void GetSFResponseFromTarget(obdref::MessageFrame &myMsg)
{
    for(size_t i=0; i < myMsg.listMessageData.size(); i++)
    {
        // build header
        obdref::ByteList headerBytes;

        if(myMsg.listMessageData[i].expHeaderBytes.size() > 0)
        {   headerBytes << myMsg.listMessageData[i].expHeaderBytes;   }
        else
        {
            if(myMsg.protocol == "ISO 15765-4 Standard")
            {   headerBytes << 0x07 << 0xE8;   }
            else if(myMsg.protocol == "ISO 15765-4 Extended")
            {   headerBytes << 0x18 << 0xDA << 0xF1 << 0x10;   }
        }

        // build data bytes
        obdref::ByteList dataBytes;
        dataBytes << 0x07;          // single frame pci byte
        dataBytes << myMsg.listMessageData[i].expDataPrefix;

        size_t addDataBytes = 8 - dataBytes.size();
        for(size_t j=0; j < addDataBytes; j++)   {
            obdref::ubyte myDataByte = obdref::ubyte(rand() % 256);
            dataBytes << myDataByte;
        }

        // save to message data
        obdref::ByteList rawData;
        rawData << headerBytes << dataBytes;
        myMsg.listMessageData[i].listRawDataFrames << rawData;
    }
}

void GetSFResponseFromRandom(obdref::MessageFrame &myMsg)
{
    for(size_t i=0; i < myMsg.listMessageData.size(); i++)
    {
        for(size_t j=0; j < 3; j++)
        {   // responses from three separate addresses

            // build header
            obdref::ByteList headerBytes;

            if(myMsg.protocol == "ISO 15765-4 Standard")
            {   headerBytes << 0x07 << 0xE8+j;   }

            else if(myMsg.protocol == "ISO 15765-4 Extended")
            {   headerBytes << 0x18 << 0xDA << 0xF1 << 0x10+j;    }

            // build data bytes
            obdref::ByteList dataBytes;
            dataBytes << 0x07;          // single frame pci byte
            dataBytes << myMsg.listMessageData[i].expDataPrefix;

            size_t addDataBytes = 8 - dataBytes.size();
            for(size_t k=0; k < addDataBytes; k++)   {
                obdref::ubyte myDataByte = obdref::ubyte(rand() % 256);
                dataBytes << myDataByte;
            }

            // save to message data
            obdref::ByteList rawData;
            rawData << headerBytes << dataBytes;
            myMsg.listMessageData[i].listRawDataFrames << rawData;
        }
    }
}

void GetMFResponseFromTarget(obdref::MessageFrame &myMsg)
{
    for(size_t i=0; i < myMsg.listMessageData.size(); i++)
    {
        // build header
        obdref::ByteList headerBytes;
        if(myMsg.listMessageData[i].expHeaderBytes.size() > 0)   {
            headerBytes << myMsg.listMessageData[i].expHeaderBytes;
        }
        else   {
            if(myMsg.protocol == "ISO 15765-4 Standard")
            {   headerBytes << 0x07 << 0xE8;   }
            else if(myMsg.protocol == "ISO 15765-4 Extended")
            {   headerBytes << 0x18 << 0xDA << 0xF1 << 0x10;   }
        }

        // build first frame
        obdref::ByteList ffDataBytes;
        ffDataBytes << 0x15 << 0x05;        // first frame pci bytes
        ffDataBytes << myMsg.listMessageData[i].expDataPrefix;

        size_t addDataBytes = 8 - ffDataBytes.size();
        for(size_t j=0; j < addDataBytes; j++)   {
            obdref::ubyte myDataByte = obdref::ubyte(rand() % 256);
            ffDataBytes << myDataByte;
        }

        obdref::ByteList ffRaw;
        ffRaw << headerBytes << ffDataBytes;
        myMsg.listMessageData[i].listRawDataFrames << ffRaw;

        // build consecutive frames
        for(size_t j=0; j < 4; j++)
        {
            obdref::ByteList cfDataBytes;
            cfDataBytes << (0x21 + j);      // consecutive frame pci bytes
            cfDataBytes << myMsg.listMessageData[i].expDataPrefix;

            addDataBytes = 8 - cfDataBytes.size();
            for(size_t k=0; k < addDataBytes; k++)   {
                obdref::ubyte myDataByte = obdref::ubyte(rand() % 256);
                cfDataBytes << myDataByte;
            }

            obdref::ByteList cfRaw;
            cfRaw << headerBytes << cfDataBytes;
            myMsg.listMessageData[i].listRawDataFrames << cfRaw;
        }

        // test: mess the order up a little bit
        myMsg.listMessageData[i].listRawDataFrames.move(0,2);
        myMsg.listMessageData[i].listRawDataFrames.move(1,3);
    }
}

void GetMFResponseFromRandom(obdref::MessageFrame &myMsg)
{
    // get response from target first
    GetMFResponseFromTarget(myMsg);

    for(size_t i=0; i < myMsg.listMessageData.size(); i++)
    {
        QList<obdref::ByteList> &listRawData =
            myMsg.listMessageData[i].listRawDataFrames;

        // modify headers to random sources
        obdref::ByteList rHeaderBytes;
        if(myMsg.protocol.contains("Extended"))   {
            rHeaderBytes << 0x18 << 0xDA << 0xF1 << obdref::ubyte(rand() % 256);
            for(size_t j=0; j < listRawData.size(); j++)   {
                listRawData[j][0] = rHeaderBytes[0];
                listRawData[j][1] = rHeaderBytes[1];
                listRawData[j][2] = rHeaderBytes[2];
                listRawData[j][3] = rHeaderBytes[3];
            }
        }
        else   {
            rHeaderBytes << 0x07 << obdref::ubyte(rand() % 256);
            for(size_t j=0; j < listRawData.size(); j++)   {
                listRawData[j][0] = rHeaderBytes[0];
                listRawData[j][1] = rHeaderBytes[1];
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

void PrintReqResp(obdref::MessageFrame const &msgFrame)
{
    qDebug() << "================================================";
    for(size_t i=0; i < msgFrame.listMessageData.size(); i++)   {
        qDebug() << "--------------------------------------------";
        qDebug() << "Parameter" << msgFrame.name;
        qDebug() << "Request:"  << msgFrame.listMessageData[i].reqHeaderBytes
                                << msgFrame.listMessageData[i].reqDataBytes;
        qDebug() << "Response:";

        QList<obdref::ByteList> const &listRawData =
                msgFrame.listMessageData[i].listRawDataFrames;

        for(size_t j=0; j < listRawData.size(); j++)   {
            qDebug() << "   " << listRawData[j];
        }
        qDebug() << "--------------------------------------------";
    }
    qDebug() << "================================================";
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
