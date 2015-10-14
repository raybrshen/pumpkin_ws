/****************************************************************************
** Meta object code from reading C++ file 'pumpkinqt.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/pumpkin_qt/include/pumpkin_qt/pumpkinqt.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pumpkinqt.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_pumpkin_qt__FolderModel[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

static const char qt_meta_stringdata_pumpkin_qt__FolderModel[] = {
    "pumpkin_qt::FolderModel\0"
};

void pumpkin_qt::FolderModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObjectExtraData pumpkin_qt::FolderModel::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject pumpkin_qt::FolderModel::staticMetaObject = {
    { &QAbstractItemModel::staticMetaObject, qt_meta_stringdata_pumpkin_qt__FolderModel,
      qt_meta_data_pumpkin_qt__FolderModel, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &pumpkin_qt::FolderModel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *pumpkin_qt::FolderModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *pumpkin_qt::FolderModel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_pumpkin_qt__FolderModel))
        return static_cast<void*>(const_cast< FolderModel*>(this));
    return QAbstractItemModel::qt_metacast(_clname);
}

int pumpkin_qt::FolderModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QAbstractItemModel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
static const uint qt_meta_data_pumpkin_qt__PumpkinQT[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      27,   23,   22,   22, 0x05,
      59,   23,   22,   22, 0x05,

 // slots: signature, parameters, type, tag, flags
     109,   89,   22,   22, 0x0a,
     174,  168,   22,   22, 0x0a,
     202,  168,   22,   22, 0x0a,
     228,   22,   22,   22, 0x0a,
     242,   22,   22,   22, 0x0a,
     263,  254,   22,   22, 0x0a,
     293,  287,   22,   22, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_pumpkin_qt__PumpkinQT[] = {
    "pumpkin_qt::PumpkinQT\0\0str\0"
    "changePlaybackFilename(QString)\0"
    "changeRecordFilename(QString)\0"
    "base_path,file_list\0"
    "fillTable(QString,std::vector<pumpkin_messages::FileList>)\0"
    "index\0folderSelected(QModelIndex)\0"
    "fileSelected(QModelIndex)\0runPlayback()\0"
    "runRecord()\0filename\0changeFilename(QString)\0"
    "state\0actionFinished(int)\0"
};

void pumpkin_qt::PumpkinQT::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PumpkinQT *_t = static_cast<PumpkinQT *>(_o);
        switch (_id) {
        case 0: _t->changePlaybackFilename((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->changeRecordFilename((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->fillTable((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const std::vector<pumpkin_messages::FileList>(*)>(_a[2]))); break;
        case 3: _t->folderSelected((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 4: _t->fileSelected((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 5: _t->runPlayback(); break;
        case 6: _t->runRecord(); break;
        case 7: _t->changeFilename((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 8: _t->actionFinished((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData pumpkin_qt::PumpkinQT::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject pumpkin_qt::PumpkinQT::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_pumpkin_qt__PumpkinQT,
      qt_meta_data_pumpkin_qt__PumpkinQT, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &pumpkin_qt::PumpkinQT::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *pumpkin_qt::PumpkinQT::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *pumpkin_qt::PumpkinQT::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_pumpkin_qt__PumpkinQT))
        return static_cast<void*>(const_cast< PumpkinQT*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int pumpkin_qt::PumpkinQT::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void pumpkin_qt::PumpkinQT::changePlaybackFilename(const QString & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void pumpkin_qt::PumpkinQT::changeRecordFilename(const QString & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
