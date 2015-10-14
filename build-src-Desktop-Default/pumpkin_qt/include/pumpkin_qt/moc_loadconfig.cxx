/****************************************************************************
** Meta object code from reading C++ file 'loadconfig.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/pumpkin_qt/include/pumpkin_qt/loadconfig.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'loadconfig.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_pumpkin_qt__LoadConfig[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      38,   24,   23,   23, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_pumpkin_qt__LoadConfig[] = {
    "pumpkin_qt::LoadConfig\0\0base_path,msg\0"
    "load(QString,std::vector<std::string>)\0"
};

void pumpkin_qt::LoadConfig::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        LoadConfig *_t = static_cast<LoadConfig *>(_o);
        switch (_id) {
        case 0: _t->load((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const std::vector<std::string>(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData pumpkin_qt::LoadConfig::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject pumpkin_qt::LoadConfig::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_pumpkin_qt__LoadConfig,
      qt_meta_data_pumpkin_qt__LoadConfig, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &pumpkin_qt::LoadConfig::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *pumpkin_qt::LoadConfig::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *pumpkin_qt::LoadConfig::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_pumpkin_qt__LoadConfig))
        return static_cast<void*>(const_cast< LoadConfig*>(this));
    return QDialog::qt_metacast(_clname);
}

int pumpkin_qt::LoadConfig::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
