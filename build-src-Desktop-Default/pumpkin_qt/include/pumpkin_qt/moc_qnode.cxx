/****************************************************************************
** Meta object code from reading C++ file 'qnode.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/pumpkin_qt/include/pumpkin_qt/qnode.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qnode.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_pumpkin_qt__QNode[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      39,   19,   18,   18, 0x05,
     113,   99,   18,   18, 0x05,
     176,  164,   18,   18, 0x05,
     207,   18,   18,   18, 0x05,

 // slots: signature, parameters, type, tag, flags
     221,   18,   18,   18, 0x0a,
     233,   18,   18,   18, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_pumpkin_qt__QNode[] = {
    "pumpkin_qt::QNode\0\0base_path,file_list\0"
    "filesReady(QString,std::vector<pumpkin_messages::FileList>)\0"
    "base_path,msg\0"
    "configFilesReady(QString,std::vector<std::string>)\0"
    "msg,timeout\0sendStatusMessage(QString,int)\0"
    "rosShutdown()\0callFiles()\0callConfigFiles()\0"
};

void pumpkin_qt::QNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QNode *_t = static_cast<QNode *>(_o);
        switch (_id) {
        case 0: _t->filesReady((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const std::vector<pumpkin_messages::FileList>(*)>(_a[2]))); break;
        case 1: _t->configFilesReady((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const std::vector<std::string>(*)>(_a[2]))); break;
        case 2: _t->sendStatusMessage((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->rosShutdown(); break;
        case 4: _t->callFiles(); break;
        case 5: _t->callConfigFiles(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData pumpkin_qt::QNode::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject pumpkin_qt::QNode::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_pumpkin_qt__QNode,
      qt_meta_data_pumpkin_qt__QNode, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &pumpkin_qt::QNode::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *pumpkin_qt::QNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *pumpkin_qt::QNode::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_pumpkin_qt__QNode))
        return static_cast<void*>(const_cast< QNode*>(this));
    return QThread::qt_metacast(_clname);
}

int pumpkin_qt::QNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void pumpkin_qt::QNode::filesReady(const QString & _t1, const std::vector<pumpkin_messages::FileList> & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void pumpkin_qt::QNode::configFilesReady(const QString & _t1, const std::vector<std::string> & _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void pumpkin_qt::QNode::sendStatusMessage(const QString & _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void pumpkin_qt::QNode::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}
QT_END_MOC_NAMESPACE
