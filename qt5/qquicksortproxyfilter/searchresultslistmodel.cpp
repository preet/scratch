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

#include "searchresultslistmodel.h"

SearchResultsListModel::SearchResultsListModel(QObject *parent) :
    QAbstractListModel(parent)
{
    m_roles[TypeRole]       = "type";
    m_roles[NameRole]       = "name";
    m_roles[RegionRole]     = "region";
    m_roles[Admin1Role]     = "admin1";
    m_roles[Admin1DispRole] = "admin1_disp";
    m_roles[DistanceRole]   = "dist_km";
    m_roles[MultipleRole]   = "multiple";

    //m_result_count = 0;
}

QHash<int,QByteArray> SearchResultsListModel::roleNames() const
{   return m_roles;   }

QVariant SearchResultsListModel::data(QModelIndex const &index, int role) const
{
    int row = index.row();

    if((row < 0) || (row >= m_list_results.size()))   {
        return QVariant();
    }

    SearchResult const * result = m_list_results[row];

    switch(role)
    {
        case TypeRole:   {
            return result->type;
        }
        case NameRole:   {
            return *(result->name);
        }
//        case RegionRole:   {
//            return result->list_regions[0];
//        }
//        case Admin1Role:   {
//            return result->list_admin1[0];
//        }
//        case Admin1DispRole:   {
//            return result->list_admin1_disp[0];
//        }
        case DistanceRole:   {
            return result->list_dist_km[0];
        }
//        case MultipleRole:   {
//            return (result->list_ids.size() > 1);
//        }
        default:
            break;
    }

    return QVariant();
}

int SearchResultsListModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return this->count();
}

void SearchResultsListModel::add(Result const &add_result)
{
    QHash<QString,SearchResult>::iterator it;
    it = m_buffer_results.find(add_result.name);
    if(it == m_buffer_results.end())   {
        SearchResult result;
        result.index = -1;
        result.type = add_result.type;
        result.list_ids.push_back(add_result.id);
        result.list_regions.push_back(add_result.region);
        result.list_admin1.push_back(add_result.admin1);
        result.list_admin1_disp.push_back(add_result.admin1_disp);
        result.list_dist_km.push_back(add_result.dist_km);
        it = m_buffer_results.insert(add_result.name,result);
    }
    else   {
        SearchResult &result = it.value();
        result.list_ids.push_back(add_result.id);
        result.list_regions.push_back(add_result.region);
        result.list_admin1.push_back(add_result.admin1);
        result.list_admin1_disp.push_back(add_result.admin1_disp);
        result.list_dist_km.push_back(add_result.dist_km);
    }
}

void SearchResultsListModel::add(quint64 id,
                                 quint8 type,
                                 QString const &name,
                                 QString const &region,
                                 QString const &admin1,
                                 bool admin1_disp,
                                 float dist_km)
{
    QHash<QString,SearchResult>::iterator it;
    it = m_buffer_results.find(name);
    if(it == m_buffer_results.end())   {
        SearchResult result;
        result.index = -1;
        result.type = type;
        result.list_ids.push_back(id);
        result.list_regions.push_back(region);
        result.list_admin1.push_back(admin1);
        result.list_admin1_disp.push_back(admin1_disp);
        result.list_dist_km.push_back(dist_km);
        it = m_buffer_results.insert(name,result);
    }
    else   {
        SearchResult &result = it.value();
        result.list_ids.push_back(id);
        result.list_regions.push_back(region);
        result.list_admin1.push_back(admin1);
        result.list_admin1_disp.push_back(admin1_disp);
        result.list_dist_km.push_back(dist_km);
    }
}

void SearchResultsListModel::commit()
{
    // the table of buffered results has
    // unique entries

    int new_inserts=0;
    QHash<QString,SearchResult>::iterator rslt_it;
    QHash<QString,SearchResult>::iterator buff_it;

    // update existing entries
    for(buff_it  = m_buffer_results.begin();
        buff_it != m_buffer_results.end(); ++buff_it)
    {
        rslt_it = m_table_name_result.find(buff_it.key());
        if(rslt_it == m_table_name_result.end())   {
            new_inserts++;
        }
        else   {
            // update existing results
            SearchResult &buf_result = buff_it.value();
            SearchResult &sav_result = rslt_it.value();
            sav_result.list_ids.append(buf_result.list_ids);
            sav_result.list_regions.append(buf_result.list_regions);
            sav_result.list_admin1.append(buf_result.list_admin1);
            sav_result.list_admin1_disp.append(buf_result.list_admin1_disp);
            sav_result.list_dist_km.append(buf_result.list_dist_km);

            emit dataChanged(this->index(sav_result.index),
                             this->index(sav_result.index));


            // mark that this buffered value has
            // already been saved
            buf_result.index = sav_result.index;
        }
    }

    if(new_inserts == 0)   {
        m_buffer_results.clear();
        return;
    }

    // insert new entries
    this->beginInsertRows(QModelIndex(),m_list_results.size(),
                          m_list_results.size()+new_inserts-1);

    for(buff_it  = m_buffer_results.begin();
        buff_it != m_buffer_results.end(); ++buff_it)
    {
        if(buff_it.value().index > -1)   {
            continue;
        }

        rslt_it = m_table_name_result.insert(buff_it.key(),buff_it.value());
        rslt_it.value().name = &(rslt_it.key());
        rslt_it.value().index = m_list_results.size();
        m_list_results.push_back(&(rslt_it.value()));
    }

    this->endInsertRows();
    emit countChanged();

    m_buffer_results.clear();
}

void SearchResultsListModel::clear()
{
    if(m_list_results.isEmpty())   {
        return;
    }

    beginRemoveRows(QModelIndex(), 0, m_list_results.size() - 1);
    m_list_results.clear();
    endRemoveRows();

    m_table_name_result.clear();
    m_buffer_results.clear();
    Q_EMIT countChanged();
}
