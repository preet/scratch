/*
   This source is Copyright 2012 Preet Desai

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

#ifndef HELPER_H
#define HELPER_H

#include <QQuickView>
#include <QQmlContext>
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QDir>
#include <QElapsedTimer>
#include <QSortFilterProxyModel>

#include "searchresultslistmodel.h"

class Helper : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString message READ getMessage NOTIFY messageChanged)

public:
    explicit Helper(QQuickView * qqview, QObject * parent=0);

    Q_INVOKABLE void searchFilterChanged(QString const &str);
    Q_INVOKABLE QObject * getSourceModel();
    
    inline QString getMessage() const   {
        QString m = "Hello World!";
        return m;
    }

signals:
    void messageChanged();

private:
    QString genRandomName(quint32 seed);

    QQuickView * m_view;
    QString m_name_letters;
    SearchResultsListModel * qml_listSearchResults;
    QSortFilterProxyModel  * qml_listSearchResultsFilter;
};

#endif // HELPER_H
