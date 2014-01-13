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

#include "helper.h"

Helper::Helper(QQuickView *qqview, QObject *parent) :
    QObject(parent)
{
    // name letter pool
    m_name_letters = "abcdefghijklmnopqrstuvwxyz";

    // search results
    qml_listSearchResults = new SearchResultsListModel(this);
    qml_listSearchResultsFilter = new QSortFilterProxyModel(this);
    qml_listSearchResultsFilter->setDynamicSortFilter(true);
    qml_listSearchResultsFilter->setSortRole(SearchResultsListModel::DistanceRole);
    qml_listSearchResultsFilter->setFilterRole(SearchResultsListModel::NameRole);
    qml_listSearchResultsFilter->setFilterCaseSensitivity(Qt::CaseInsensitive);
    qml_listSearchResultsFilter->setSourceModel(qml_listSearchResults);

    // populate results
    for(int i=0; i < 2500; i++)   {
        SearchResultsListModel::Result r;
        r.name = genRandomName(i);
        r.dist_km = i+1;
        qml_listSearchResults->add(r);
    }
    qml_listSearchResults->commit();
    qml_listSearchResultsFilter->sort(0);

    m_view = qqview;
    m_view->rootContext()->setContextProperty("Helper",this);
    m_view->rootContext()->setContextProperty("SearchResultsModel",qml_listSearchResultsFilter);
    m_view->setResizeMode(QQuickView::SizeRootObjectToView);
    m_view->setSource(QUrl("qrc:/main.qml"));
    m_view->show();
}

void Helper::searchFilterChanged(QString const &str)
{
//    qml_listSearchResultsFilter->setFilterFixedString(str);
    qml_listSearchResultsFilter->setFilterRegExp(str);
//    qml_listSearchResultsFilter->sort(0);
}

QObject * Helper::getSourceModel()
{
    return qml_listSearchResults;
}

QString Helper::genRandomName(quint32 seed)
{
    srand(seed);
    int length = (rand()%12) + 5;

    QString name;
    for(int i=0; i < length; i++)   {
        int index = rand()%m_name_letters.size();
        name.push_back(m_name_letters[index]);
    }
    return name;
}
