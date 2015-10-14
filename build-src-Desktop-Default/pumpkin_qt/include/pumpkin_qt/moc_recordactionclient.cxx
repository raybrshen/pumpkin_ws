/****************************************************************************
** Meta object code from reading C++ file 'recordactionclient.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/pumpkin_qt/include/pumpkin_qt/recordactionclient.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'recordactionclient.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_pumpkin_qt__RecordActionClient[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      40,   32,   31,   31, 0x05,
      74,   66,   31,   31, 0x05,
     106,  100,   31,   31, 0x05,
     132,  126,   31,   31, 0x05,
     163,  151,   31,   31, 0x05,

 // slots: signature, parameters, type, tag, flags
     194,   31,   31,   31, 0x0a,
     210,  201,   31,   31, 0x0a,
     237,   32,   31,   31, 0x0a,
     263,   66,   31,   31, 0x0a,
     289,   31,   31,   31, 0x0a,
     302,   31,   31,   31, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_pumpkin_qt__RecordActionClient[] = {
    "pumpkin_qt::RecordActionClient\0\0minutes\0"
    "recordMinuteFeedback(int)\0seconds\0"
    "recordSecondFeedback(int)\0state\0"
    "recordFinished(int)\0block\0blockPlayTab(bool)\0"
    "msg,timeout\0sendStatusMessage(QString,int)\0"
    "init()\0filename\0setRecordFilename(QString)\0"
    "setRecordTimeMinutes(int)\0"
    "setRecordTimeSeconds(int)\0recordFile()\0"
    "recordStop()\0"
};

void pumpkin_qt::RecordActionClient::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        RecordActionClient *_t = static_cast<RecordActionClient *>(_o);
        switch (_id) {
        case 0: _t->recordMinuteFeedback((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->recordSecondFeedback((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->recordFinished((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->blockPlayTab((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->sendStatusMessage((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 5: _t->init(); break;
        case 6: _t->setRecordFilename((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 7: _t->setRecordTimeMinutes((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->setRecordTimeSeconds((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->recordFile(); break;
        case 10: _t->recordStop(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData pumpkin_qt::RecordActionClient::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject pumpkin_qt::RecordActionClient::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_pumpkin_qt__RecordActionClient,
      qt_meta_data_pumpkin_qt__RecordActionClient, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &pumpkin_qt::RecordActionClient::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *pumpkin_qt::RecordActionClient::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *pumpkin_qt::RecordActionClient::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_pumpkin_qt__RecordActionClient))
        return static_cast<void*>(const_cast< RecordActionClient*>(this));
    return QObject::qt_metacast(_clname);
}

int pumpkin_qt::RecordActionClient::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void pumpkin_qt::RecordActionClient::recordMinuteFeedback(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void pumpkin_qt::RecordActionClient::recordSecondFeedback(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void pumpkin_qt::RecordActionClient::recordFinished(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void pumpkin_qt::RecordActionClient::blockPlayTab(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void pumpkin_qt::RecordActionClient::sendStatusMessage(const QString & _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
