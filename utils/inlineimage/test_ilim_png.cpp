/*
   Copyright (C) 2014 Preet Desai (preet.desai@gmail.com)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <QGuiApplication>
#include <QQuickView>
#include <QDebug>
#include <QImage>

#include <test_ilim_png.h>

#include <ilim_png.hpp>

static void myImageCleanupHandler(void * info)
{
    //qDebug() << "image cleanup";
    std::vector<uint8_t> * data =
            static_cast<std::vector<uint8_t>*>(info);
    delete data;
}

ImageProvider::ImageProvider() :
    QQuickImageProvider(QQuickImageProvider::Image)
{
    // empty
}

QImage ImageProvider::requestImage(QString const &id,
                                   QSize * size,
                                   QSize const &requestedSize)
{
    (void)id;
    (void)requestedSize;

    if(size) {
        *size = QSize(32,32);
    }

    static const QString path =
            "/home/preet/Dev/scratch/"
            "utils/inlineimage/test_images/";

    QString pixel_type = id.mid(id.lastIndexOf("_")+1);
    QString image_file = id.left(id.lastIndexOf("_"));
    //qDebug() << "pixel_type: " << pixel_type;
    //qDebug() << "image_file: " << image_file;

    std::vector<uint8_t> * qimage_data =
            new std::vector<uint8_t>;

    qimage_data->reserve(32*32*4);

    bool format_match;

    if(pixel_type == "r8") {
        ilim::Image<ilim::R8> ilim_image;
        ilim::load_png((path+image_file).toStdString(),ilim_image,&format_match);

        // test conv_to_image_data and set(ImageData &)
        ilim::ImageData image_data = ilim_image.conv_to_image_data();
        ilim_image.set(image_data);

        assert(ilim_image.data().size() == 32*32);
        for(size_t i=0; i < ilim_image.data().size(); i++) {
            ilim::RGBA8 pixel = { ilim_image.data()[i].r,0,0,255 };
            qimage_data->push_back(pixel.r);
            qimage_data->push_back(pixel.g);
            qimage_data->push_back(pixel.b);
            qimage_data->push_back(pixel.a);
        }
    }
    else if(pixel_type == "rgb8") {
        ilim::Image<ilim::RGB8> ilim_image;
        ilim::load_png((path+image_file).toStdString(),ilim_image,&format_match);

        // test conv_to_image_data and set(ImageData &)
        ilim::ImageData image_data = ilim_image.conv_to_image_data();
        ilim_image.set(image_data);

        assert(ilim_image.data().size() == 32*32);
        for(size_t i=0; i < ilim_image.data().size(); i++) {
            qimage_data->push_back(ilim_image.data()[i].r);
            qimage_data->push_back(ilim_image.data()[i].g);
            qimage_data->push_back(ilim_image.data()[i].b);
            qimage_data->push_back(255);
        }
    }
    else if(pixel_type == "rgba8") {
        ilim::Image<ilim::RGBA8> ilim_image;
        ilim::load_png((path+image_file).toStdString(),ilim_image,&format_match);

        // test conv_to_image_data and set(ImageData &)
        ilim::ImageData image_data = ilim_image.conv_to_image_data();
        ilim_image.set(image_data);

        assert(ilim_image.data().size() == 32*32);
        for(size_t i=0; i < ilim_image.data().size(); i++) {
            qimage_data->push_back(ilim_image.data()[i].r);
            qimage_data->push_back(ilim_image.data()[i].g);
            qimage_data->push_back(ilim_image.data()[i].b);
            qimage_data->push_back(ilim_image.data()[i].a);
        }
    }
    else {
        for(size_t i=0; i < 32*32; i++) {
            qimage_data->push_back(255); // r
            qimage_data->push_back(0);   // g
            qimage_data->push_back(255); // b
            qimage_data->push_back(255); // a
        }
    }

    QImage qimage(&(*qimage_data)[0],32,32,
            QImage::Format_RGBA8888,
            myImageCleanupHandler,
            qimage_data);

    QColor format_color;
    if(format_match) {
        format_color.setRgb(0,255,0,255);
    }
    else {
        format_color.setRgb(255,0,0,255);
    }

    for(size_t i=0; i < 6; i++) {
        for(size_t j=0; j < 6; j++) {
            qimage.setPixel(i,j,format_color.rgba());
        }
    }

    return qimage;
}

//QImage ImageProvider::requestImage(QString const &id,
//                                   QSize * size,
//                                   QSize const &requestedSize)
//{
//    (void)id;
//    (void)requestedSize;

//    if(size) {
//        *size = QSize(32,32);
//    }

//    std::vector<uint8_t> * qimage_data =
//            new std::vector<uint8_t>;

//    qimage_data->reserve(32*32*4);

//    for(size_t i=0; i < 32*32; i++) {
//        ilim::RGBA8 pixel = { 0,255,0,255 };
//        qimage_data->push_back(pixel.r);
//        qimage_data->push_back(pixel.g);
//        qimage_data->push_back(pixel.b);
//        qimage_data->push_back(pixel.a);
//    }

//    QImage qimage(&(*qimage_data)[0],32,32,
//            QImage::Format_RGBA8888,
//            myImageCleanupHandler,
//            qimage_data);

//    return qimage;
//}


int main(int argc, char **argv)
{
    // create image provider
    ImageProvider * image_provider = new ImageProvider();

    QGuiApplication app(argc,argv);
    QQuickView * qqview = new QQuickView();
    qqview->engine()->addImageProvider("test_png", image_provider);
    qqview->setResizeMode(QQuickView::SizeRootObjectToView);
    qqview->setSource(QUrl("/home/preet/Dev/scratch/utils/inlineimage/test_ilim_png.qml"));
    qqview->show();

    int retval = app.exec();
    delete qqview; // does this clean up the image provider too?

    return retval;
}
