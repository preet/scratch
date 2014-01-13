#ifndef QSORTROLEOBJECTLISTMODEL_H
#define QSORTROLEOBJECTLISTMODEL_H

#include <QAbstractListModel>

class QSortRoleObjectListModel : public QAbstractListModel
{
    Q_OBJECT
    Q_PROPERTY(int count READ count NOTIFY countChanged)

public:
    enum Roles   {
        ROLE_OBJECT = Qt::UserRole+1,
        ROLE_SORT   = Qt::UserRole+2
    };

    explicit QSortRoleObjectListModel(QString const &sortPropertyName,
                                      QObject *parent=0);

    QHash<int,QByteArray> roleNames() const;
    QVariant data(QModelIndex const &index, int role) const;
    int rowCount(QModelIndex const &parent) const;

    void append(QObject *object);
    void append(QObjectList const &objects);
    void removeAt(int i,int count = 1);
    void clear();

    inline QString sortProperty() const {
        return QString(m_sort_property);
    }
    inline QObject *at(int i) const {
        return m_list_objects.at(i);
    }
    inline QObject *operator[](int i) const {
        return m_list_objects[i];
    }
    inline bool contains(QObject *object) const  {
        return m_list_objects.contains(object);
    }
    inline int indexOf (QObject *object, int from = 0) const {
        return m_list_objects.indexOf(object, from);
    }
    inline int count() const  {
        return m_list_objects.count();
    }
    inline int size() const {
        return m_list_objects.size();
    }
    inline bool isEmpty() const  {
        return m_list_objects.isEmpty();
    }

Q_SIGNALS:
    void countChanged();

private:
    Q_DISABLE_COPY(QSortRoleObjectListModel)

    QByteArray m_sort_property;
    QHash<int,QByteArray> m_roles;
    QObjectList m_list_objects;
};

#endif // QSORTROLEOBJECTLISTMODEL_H
