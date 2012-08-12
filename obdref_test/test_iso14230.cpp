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

void GetTypeAResponseFromTarget(obdref::MessageFrame &myMsg);
void GetTypeBResponseFromTarget(obdref::MessageFrame &myMsg);
void GetTypeCResponseFromTarget(obdref::MessageFrame &myMsg);
void GetTypeDResponseFromTarget(obdref::MessageFrame &myMsg);

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
                                     "ISO 14230-4",
                                     "Default");
//    myParamList.removeLast();
//    myParamList.removeLast();

    for(int i=0; i < myParamList.size(); i++)
    {
        QList<obdref::Data> listData;

        // create message frame
        obdref::MessageFrame myMsg;
        myMsg.spec = "SAEJ1979";
        myMsg.protocol = "ISO 14230-4";
        myMsg.address = "Default";
        myMsg.name = myParamList.at(i);

        // build a message frame
        opOk = myParser.BuildMessageFrame(myMsg);
        if(!opOk)   {
            qDebug() << "BuildMessageFrame for"
                     << myParamList.at(i)
                     << "Failed! Exiting...";
            PrintErrors(myParser);
            return -1;
        }

        // simulate vehicle response

        GetTypeDResponseFromTarget(myMsg);
        GetTypeBResponseFromTarget(myMsg);
//        PrintReqResp(myMsg);

        // parse message frame
        opOk = myParser.ParseMessageFrame(myMsg,listData);
        if(!opOk)   {
            PrintErrors(myParser);
            return -1;
        }

        // print out data
//        PrintData(listData);
    }

    return 0;
}

void GetTypeAResponseFromTarget(obdref::MessageFrame &myMsg)
{
    // header: A [format]
    // data: 1-63 bytes
    for(size_t i=0; i < myMsg.listMessageData.size(); i++)
    {
        obdref::ByteList dataBytes;
        dataBytes << myMsg.listMessageData[i].expDataPrefix;

        // we set a min of 4 bytes, max of 63 bytes
        obdref::ubyte numAddBytes = (rand() % 59 + 4) - dataBytes.size();
        for(size_t j=0; j < numAddBytes; j++)   {
            dataBytes << obdref::ubyte(rand() % 256);
        }

        obdref::ByteList headerBytes;
        headerBytes << 0x80 + dataBytes.size();   // format byte 0b10000000

        obdref::ByteList allBytes;
        allBytes << headerBytes << dataBytes;
        myMsg.listMessageData[i].listRawDataFrames << allBytes;
    }
}

void GetTypeBResponseFromTarget(obdref::MessageFrame &myMsg)
{
    // B [format] [target] [source]
    // data 1-63 bytes
    for(size_t i=0; i < myMsg.listMessageData.size(); i++)
    {
        obdref::ByteList dataBytes;
        dataBytes << myMsg.listMessageData[i].expDataPrefix;

        // we set a min of 4 bytes, max of 63 bytes
        obdref::ubyte numAddBytes = (rand() % 59 + 4) - dataBytes.size();
        for(size_t j=0; j < numAddBytes; j++)   {
            dataBytes << obdref::ubyte(rand() % 256);
        }

        obdref::ByteList headerBytes;
        headerBytes << 0x80 + dataBytes.size();                    // format byte 0b10000000
        headerBytes << myMsg.listMessageData[i].expHeaderBytes[1]; // target byte
        headerBytes << myMsg.listMessageData[i].expHeaderBytes[2]; // source byte

        obdref::ByteList allBytes;
        allBytes << headerBytes << dataBytes;
        myMsg.listMessageData[i].listRawDataFrames << allBytes;
    }
}

void GetTypeCResponseFromTarget(obdref::MessageFrame &myMsg)
{
    // C [format] [length]
    // data 1-255 bytes
    for(size_t i=0; i < myMsg.listMessageData.size(); i++)
    {
        obdref::ByteList dataBytes;
        dataBytes << myMsg.listMessageData[i].expDataPrefix;

        // we set a min of 4 bytes, max of 255 bytes
        obdref::ubyte numAddBytes = (rand() % 251 + 4) - dataBytes.size();

        for(size_t j=0; j < numAddBytes; j++)   {
            dataBytes << obdref::ubyte(rand() % 256);
        }

        obdref::ByteList headerBytes;
        headerBytes << 0x80;           // format byte 0b10000000
        headerBytes << dataBytes.size();    // length byte

        obdref::ByteList allBytes;
        allBytes << headerBytes << dataBytes;
        myMsg.listMessageData[i].listRawDataFrames << allBytes;
    }
}

void GetTypeDResponseFromTarget(obdref::MessageFrame &myMsg)
{
    // D [format] [source] [target] [length]
    // data 1-255 bytes
    for(size_t i=0; i < myMsg.listMessageData.size(); i++)
    {
        obdref::ByteList dataBytes;
        dataBytes << myMsg.listMessageData[i].expDataPrefix;

        // we set a min of 4 bytes, max of 255 bytes
        obdref::ubyte numAddBytes = (rand() % 251 + 4) - dataBytes.size();

        for(size_t j=0; j < numAddBytes; j++)   {
            dataBytes << obdref::ubyte(rand() % 256);
        }

        obdref::ByteList headerBytes;
        headerBytes << 0x80;                                       // format byte 0b10000000
        headerBytes << myMsg.listMessageData[i].expHeaderBytes[1]; // target byte
        headerBytes << myMsg.listMessageData[i].expHeaderBytes[2] + 1; // source byte
        headerBytes << dataBytes.size();                             // length byte

        obdref::ByteList allBytes;
        allBytes << headerBytes << dataBytes;
        myMsg.listMessageData[i].listRawDataFrames << allBytes;
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

void PrintResponse(obdref::MessageFrame const &msgFrame)
{

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
