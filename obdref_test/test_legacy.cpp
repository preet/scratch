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

void GetResponseFromTarget(obdref::MessageFrame &myMsg);
void GetResponseFromRandom(obdref::MessageFrame &myMsg);
void GetResponseWithHeader(obdref::MessageFrame &myMsg,
                           obdref::ByteList headerBytes);

void PrintErrors(obdref::Parser &myParser);
void PrintData(QList<obdref::Data> &listData);
void PrintReqResp(const obdref::MessageFrame &msgFrame);

int main(int argc, char* argv[])
{
    // expect argument: path to definitions file
    QString filePath(argv[1]);
    if(filePath.isEmpty())   {
       qDebug() << "OBDREF: Error: Pass in the definitions "
                   "file as an argument";
       return -1;
    }


    // read in definitions file
    bool opOk = false;
    obdref::Parser myParser(filePath,opOk);
    if(!opOk)   {   PrintErrors(myParser);   return -1;   }
    qDebug() << "OBDREF: Info: Successfully read definitions";


    // get a list of default parameters
    QStringList myParamList
        = myParser.GetParameterNames("SAEJ1979","ISO 9141-2","Default");


    for(size_t i=0; i < myParamList.size(); i++)
    {
        QList<obdref::Data> listData;

        // create message frame
        obdref::MessageFrame myMsg;
        myMsg.spec = "SAEJ1979";
        myMsg.protocol = "ISO 9141-2";
        myMsg.address = "Default";
        myMsg.name = myParamList.at(i);

        // build a message frame
        opOk = myParser.BuildMessageFrame(myMsg);
        if(!opOk)   {   PrintErrors(myParser);   return -1;   }

        // simulate vehicle response
        GetResponseFromTarget(myMsg);

        // parse message frame
        opOk = myParser.ParseMessageFrame(myMsg,listData);
        if(!opOk)   {   PrintErrors(myParser);   return -1;   }

        // print out data
        PrintData(listData);
    }

    return 0;
}

void GetResponseFromTarget(obdref::MessageFrame &myMsg)
{
    for(size_t i=0; i < myMsg.listMessageData.size(); i++)
    {
        // build header
        obdref::ByteList headerBytes;
        headerBytes << 0x48 << 0x6B << 0x10;
        if(myMsg.listMessageData[i].expHeaderBytes.size() > 0)   {
            headerBytes[1] = myMsg.listMessageData[i].expHeaderBytes[1];
        }

        // build databytes
        obdref::ByteList dataBytes;
        dataBytes << myMsg.listMessageData[i].expDataPrefix;

        size_t numAddBytes = 7 - dataBytes.size();
        for(size_t j=0; j < numAddBytes; j++)   {
            dataBytes << obdref::ubyte(rand() % 256);
        }

        // save
        obdref::ByteList rawData;
        rawData << headerBytes << dataBytes;
        myMsg.listMessageData[i].listRawDataFrames << rawData;
    }
}

void GetResponseFromRandom(obdref::MessageFrame &myMsg)
{
    GetResponseFromTarget(myMsg);
    for(size_t i=0; i < myMsg.listMessageData.size(); i++)   {
        for(size_t j=0; j < myMsg.listMessageData[i].listRawDataFrames.size(); j++)   {
            obdref::ubyte randomByte = obdref::ubyte(rand() % 256);
            myMsg.listMessageData[i].listRawDataFrames[j][2] = randomByte;
        }
    }
}

void GetResponseWithHeader(obdref::MessageFrame &myMsg,
                           obdref::ByteList headerBytes)
{
    GetResponseFromTarget(myMsg);
    for(size_t i=0; i < myMsg.listMessageData.size(); i++)   {
        for(size_t j=0; j < myMsg.listMessageData[i].listRawDataFrames.size(); j++)   {
            myMsg.listMessageData[i].listRawDataFrames[j][0] = headerBytes[0];
            myMsg.listMessageData[i].listRawDataFrames[j][1] = headerBytes[1];
            myMsg.listMessageData[i].listRawDataFrames[j][2] = headerBytes[2];
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
