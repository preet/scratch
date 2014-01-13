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

#ifndef SEARCH_RESULTS_LISTMODEL_H
#define SEARCH_RESULTS_LISTMODEL_H

#include <QAbstractListModel>


class SearchResultsListModel : public QAbstractListModel
{
    Q_OBJECT
    Q_PROPERTY(int count READ count NOTIFY countChanged)

    struct SearchResult
    {
        int index;

        QString const * name;
        quint8 type;

        QList<quint64> list_ids;
        QList<QString> list_regions;
        QList<QString> list_admin1;
        QList<bool>    list_admin1_disp;
        QList<float>   list_dist_km;

        SearchResult() :
            index(-1),name(NULL) {}
    };

public:
    struct Result
    {
        quint64 id;
        quint8 type;
        QString name;
        QString region;
        QString admin1;
        bool admin1_disp;
        float dist_km;
    };

    // model roles
    enum Roles   {
        TypeRole        = Qt::UserRole+1,
        NameRole        = Qt::UserRole+2,
        RegionRole      = Qt::UserRole+3,
        Admin1Role      = Qt::UserRole+4,
        Admin1DispRole  = Qt::UserRole+5,
        DistanceRole    = Qt::UserRole+6,
        MultipleRole    = Qt::UserRole+7
    };

    explicit SearchResultsListModel(QObject *parent = 0);

    QHash<int,QByteArray> roleNames() const;

    QVariant data(const QModelIndex &index, int role) const;
    int rowCount(const QModelIndex &parent) const;

    void add(Result const &add_result);

    void add(quint64 id,
             quint8 type,
             QString const &name,
             QString const &region,
             QString const &admin1,
             bool admin1_disp,
             float dist_km);

    void commit();
    void clear();

    inline int count() const
    {
        return m_list_results.count();
    }

    inline int size() const
    {
        return m_list_results.size();
    }

    inline int isEmpty() const
    {
        return m_list_results.isEmpty();
    }

    inline int resultCount() const
    {   // TODO
        return 0;
    }

Q_SIGNALS:
    void countChanged();

private:
    Q_DISABLE_COPY(SearchResultsListModel)

    QHash<int,QByteArray> m_roles;

    // apparently all keys remain intact unless
    // the element is removed, so it should be
    // safe to maintain a pointer to individual
    // keys in SearchResult

    QHash<QString,SearchResult> m_table_name_result;
    QHash<QString,SearchResult> m_buffer_results;
    QList<SearchResult const *> m_list_results;

    //
    quint64 m_memory_used;
};


#endif // SEARCH_RESULTS_LISTMODEL_H
