#include "qsortedobjectlist.h"

QSortRoleObjectListModel::QSortRoleObjectListModel(QString const &sortPropertyName,
                                                   QObject *parent) :
    QAbstractListModel(parent)
{
    m_roles[ROLE_OBJECT] = "object";
    m_roles[ROLE_SORT]   = "_private_sort_role";

    m_sort_property = sortPropertyName.toLocal8Bit();
}

// ============================================================== //

QHash<int,QByteArray> QSortRoleObjectListModel::roleNames() const
{   return m_roles;   }

QVariant QSortRoleObjectListModel::data(QModelIndex const &index, int role) const
{
    int row = index.row();

    if((row < 0) || (row >= m_list_objects.size()))   {
        return QVariant();
    }

    if(role == ROLE_OBJECT)   {
        return QVariant::fromValue(m_list_objects.at(row));
    }
    else if(role == ROLE_SORT)   {
        QObject * obj = m_list_objects.at(index.row());
        QVariant sort_prop = obj->property(m_sort_property.data());
        if(!sort_prop.isNull())   {
            return sort_prop;
        }
    }

    return QVariant();
}

int QSortRoleObjectListModel::rowCount(QModelIndex const &parent) const
{
    Q_UNUSED(parent);
    return this->count();
}

// ============================================================== //

void QSortRoleObjectListModel::append(QObject *object)
{
    beginInsertRows(QModelIndex(),
                    m_list_objects.count(),
                    m_list_objects.count());
    m_list_objects.append(object);
    endInsertRows();
    this->sort();
    Q_EMIT countChanged();
}

void QSortRoleObjectListModel::append(QObjectList const &objects)
{
    beginInsertRows(QModelIndex(),
                    m_list_objects.count(),
                    m_list_objects.count()+objects.count()-1);
    m_list_objects.append(objects);
    endInsertRows();
    Q_EMIT countChanged();
}


void QSortRoleObjectListModel::removeAt(int i,int count)
{
    beginRemoveRows(QModelIndex(),i,i+count-1);
    for (int j = 0; j < count; ++j)   {
        m_list_objects.removeAt(i);
    }
    endRemoveRows();
    Q_EMIT countChanged();
}

void QSortRoleObjectListModel::clear()
{
    if (m_list_objects.isEmpty())   {
        return;
    }

    beginRemoveRows(QModelIndex(),0,m_list_objects.count()-1);
    m_list_objects.clear();
    endRemoveRows();
    Q_EMIT countChanged();
}
