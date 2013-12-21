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

#ifndef QOBJECTLISTMODEL_H
#define QOBJECTLISTMODEL_H

#include <QAbstractListModel>

/*
    Open issues:
        object ownership: is it helpful for the model to own the objects?
                          can we guard the objects so they are automatically removed
                              from the model when deleted?
        add additional QList convenience functions (operator<<, etc.)
*/

class QObjectListModel : public QAbstractListModel
{
    Q_OBJECT
    Q_PROPERTY(int count READ count NOTIFY countChanged)
public:
    explicit QObjectListModel(QObject *parent = 0);
    QObjectListModel(QObjectList objects, QObject *parent = 0);

    //model API
    enum Roles { ObjectRole = Qt::UserRole+1 };

    QHash<int, QByteArray> roleNames() const;

    int rowCount(const QModelIndex &parent) const;
    QVariant data(const QModelIndex &index, int role) const;

    QObjectList objectList() const;
    void setObjectList(QObjectList objects);

    //list API
    void append(QObject *object);
    void append(const QObjectList &objects);
    void insert(int i, QObject *object);
    void insert(int i, const QObjectList &objects);

    inline QObject *at(int i) const { return m_objects.at(i); }
    inline QObject *operator[](int i) const { return m_objects[i]; }
    void replace(int i, QObject *object);

    void move(int from, int to);

    void removeAt(int i, int count = 1);
    QObject *takeAt(int i);
    void clear();

    inline bool contains(QObject *object) const { return m_objects.contains(object); }
    inline int indexOf (QObject *object, int from = 0) const { return m_objects.indexOf(object, from); }
    inline int lastIndexOf (QObject *object, int from = -1) const { return m_objects.lastIndexOf(object, from); }

    inline int count() const { return m_objects.count(); }
    inline int size() const { return m_objects.size(); }
    inline bool isEmpty() const { return m_objects.isEmpty(); }

    inline QObjectList::Iterator begin() { return m_objects.begin(); }
    inline QObjectList::Iterator end() { return m_objects.end(); }

    //additional QML API
    Q_INVOKABLE QObject *get(int i) const;

Q_SIGNALS:
    void countChanged();

private:
    Q_DISABLE_COPY(QObjectListModel)
    QObjectList m_objects;
};

#endif // QOBJECTMODEL_H
