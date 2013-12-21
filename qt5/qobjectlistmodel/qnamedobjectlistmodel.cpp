/****************************************************************************
**
** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the QML project on Qt Labs.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
**     the names of its contributors may be used to endorse or promote
**     products derived from this software without specific prior written
**     permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
** $QT_END_LICENSE$
**
****************************************************************************/

#include "qnamedobjectlistmodel.h"

/*!
    \class QNamedObjectListModel
    \brief The QNamedObjectListModel class provides a model that supplies objects to QML views.

    QNamedObjectListModel provides a more powerful, but still easy to use, alternative to using
    QObjectList lists as models for QML views. As a QAbstractListModel, it has the ability to
    automatically notify the view of specific changes to the list, such as adding or removing
    items. At the same time it provides QList-like convenience functions such as append, at,
    and removeAt for easily working with the model from C++.

    \code
    QNamedObjectListModel model;
    model.setObjectList(myQList);
    model.append(myNewObject);
    ...
    int pos = model.indexOf(myObject);
    model.insert(pos, myOtherNewObject);
    ...
    model.removeAt(0);
    \endcode

    QNamedObjectListModel exposes a single \c object role to QML,
    as well as a \c count property, and a \c get(int i) function.

    \qml
    ListView {
        ...
        delegate: Text { text: object.someProperty }
    }
    \endqml
*/

namespace mapmixview
{

/*!
    Constructs an object list model with the given \a parent.
*/
QNamedObjectListModel::QNamedObjectListModel(QObject *parent) :
    QAbstractListModel(parent) {}

/*!
    Constructs an object list model containing the specified \a objects
    with the given \a parent.
*/
QNamedObjectListModel::QNamedObjectListModel(QObjectList objects, QObject *parent) :
    QAbstractListModel(parent), m_objects(objects) {}

QHash<int, QByteArray> QNamedObjectListModel::roleNames() const {
    QHash<int, QByteArray> roles;
    roles[ObjectRole] = "object";
    roles[ObjectNameRole] = "objectName";
    return roles;
}

/*!
    Returns data for the specified \a role, from the item with the
    given \a index. The only valid role is \c ObjectRole.

    If the view requests an invalid index or role, an invalid variant
    is returned.
*/
QVariant QNamedObjectListModel::data(const QModelIndex &index, int role) const
{
    if (index.row() < 0 || index.row() >= m_objects.size())
        return QVariant();

    if (role == ObjectRole) {
        return QVariant::fromValue(m_objects.at(index.row()));
    }
    else if(role == ObjectNameRole)   {
        QObject * obj = m_objects.at(index.row());
        return obj->property("name");
    }

    return QVariant();
}

/*!
    Returns the number of rows in the model. This value corresponds to the
    number of items in the model's internal object list.
*/
int QNamedObjectListModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return count();
}

/*!
    Returns the object list used by the model to store data.
*/
QObjectList QNamedObjectListModel::objectList() const
{
    return m_objects;
}

/*!
    Sets the model's internal objects list to \a objects. The model will
    notify any attached views that its underlying data has changed.
*/
void QNamedObjectListModel::setObjectList(QObjectList objects)
{
    int oldCount = m_objects.count();
    beginResetModel();
    m_objects = objects;
    endResetModel();
    Q_EMIT dataChanged(index(0), index(m_objects.count()));
    if (m_objects.count() != oldCount)
        Q_EMIT countChanged();
}

/*!
    Inserts \a object at the end of the model and notifies any views.

    This is the same as model.insert(size(), \a object).

    \sa insert()
*/
void QNamedObjectListModel::append(QObject *object)
{
    beginInsertRows(QModelIndex(), m_objects.count(), m_objects.count());
    m_objects.append(object);
    endInsertRows();
    Q_EMIT countChanged();
}

/*!
    \overload
    Appends the items of the \a objects list to this model and notifies any views.
*/
void QNamedObjectListModel::append(const QObjectList &objects)
{
    beginInsertRows(QModelIndex(), m_objects.count(), m_objects.count()+objects.count()-1);
    m_objects.append(objects);
    endInsertRows();
    Q_EMIT countChanged();
}

/*!
    Inserts \a object at index position \a i in the model and notifies
    any views. If \a i is 0, the object is prepended to the model. If \a i
    is size(), the object is appended to the list.

    \sa append(), replace(), removeAt()
*/
void QNamedObjectListModel::insert(int i, QObject *object)
{
    beginInsertRows(QModelIndex(), i, i);
    m_objects.insert(i, object);
    endInsertRows();
    Q_EMIT countChanged();
}

/*!
    \overload
    Inserts the items of the \a objects list at index position \a i in the model
    and notifies any views. If \a i is 0, the items are prepended to the model. If \a i
    is size(), the items are appended to the list.
*/
void QNamedObjectListModel::insert(int i, const QObjectList &objects)
{
    if (objects.isEmpty())
        return;

    beginInsertRows(QModelIndex(), i, i+objects.count()-1);
    for (int j = objects.count() - 1; j > -1; --j)
        m_objects.insert(i, objects.at(j));
    endInsertRows();
    Q_EMIT countChanged();
}

/*!
    Replaces the item at index position \a i with \a object and
    notifies any views. \a i must be a valid index position in the list
    (i.e., 0 <= \a i < size()).

    \sa removeAt()
*/
void QNamedObjectListModel::replace(int i, QObject *object)
{
    m_objects.replace(i, object);
    Q_EMIT dataChanged(index(i), index(i));
}

/*!
    Moves the item at index position \a from to index position \a to
    and notifies any views.

    This function
    assumes that both \a from and \a to are at least 0 but less than
    size(). To avoid failure, test that both \a from and \a to are at
    least 0 and less than size().
*/

void QNamedObjectListModel::move(int from, int to)
{
    if (!beginMoveRows(QModelIndex(), from, from, QModelIndex(), to > from ? to+1 : to))
        return; //should only be triggered for our simple case if from == to.
    m_objects.move(from, to);
    endMoveRows();
}

/*!
    Removes \a count number of items from index position \a i and notifies any views.
    \a i must be a valid index position in the model (i.e., 0 <= \a i < size()), as
    must \c{i + count - 1}.

    \sa takeAt()
*/
void QNamedObjectListModel::removeAt(int i, int count)
{
    beginRemoveRows(QModelIndex(), i, i + count - 1);
    for (int j = 0; j < count; ++j)
        m_objects.removeAt(i);
    endRemoveRows();
    Q_EMIT countChanged();
}

/*!
    Removes the item at index position \a i (notifying any views) and returns it.
    \a i must be a valid index position in the model (i.e., 0 <= \a i < size()).

    \sa removeAt()
*/
QObject *QNamedObjectListModel::takeAt(int i)
{
    beginRemoveRows(QModelIndex(), i, i);
    QObject *obj = m_objects.takeAt(i);
    endRemoveRows();
    Q_EMIT countChanged();
    return obj;
}

/*!
    Removes all items from the model and notifies any views.
*/
void QNamedObjectListModel::clear()
{
    if (m_objects.isEmpty())
        return;

    beginRemoveRows(QModelIndex(), 0, m_objects.count() - 1);
    m_objects.clear();
    endRemoveRows();
    Q_EMIT countChanged();
}

/*!
    \internal
    For usage from QML.
*/
QObject *QNamedObjectListModel::get(int i) const
{
    return m_objects.at(i);
}

/*!
    \fn int QNamedObjectListModel::size() const

    Returns the number of items in the model.

    \sa isEmpty(), count()
*/

/*! \fn int QNamedObjectListModel::count() const

    Returns the number of items in the model. This is effectively the
    same as size().
*/

/*! \fn bool QNamedObjectListModel::isEmpty() const

    Returns true if the model contains no items; otherwise returns
    false.

    \sa size()
*/

/*! \fn QObject *QNamedObjectListModel::at(int i) const

    Returns the object at index position \a i in the list. \a i must be
    a valid index position in the model (i.e., 0 <= \a i < size()).

    \sa operator[]()
*/

/*! \fn QObject *QNamedObjectListModel::operator[](int i) const

    \overload

    Same as at().
*/

/*! \fn int QNamedObjectListModel::indexOf(QObject *object, int from = 0) const

    Returns the index position of the first occurrence of \a object in
    the model, searching forward from index position \a from. Returns
    -1 if no item matched.

    \sa lastIndexOf(), contains()
*/

/*! \fn int QNamedObjectListModel::lastIndexOf(QObject *object, int from = -1) const

    Returns the index position of the last occurrence of \a object in
    the list, searching backward from index position \a from. If \a
    from is -1 (the default), the search starts at the last item.
    Returns -1 if no item matched.

    \sa indexOf()
*/

/*! \fn bool QNamedObjectListModel::contains(Object *object) const

    Returns true if the list contains an occurrence of \a object;
    otherwise returns false.

    \sa indexOf(), count()
*/

}   // namespace mapmixview
