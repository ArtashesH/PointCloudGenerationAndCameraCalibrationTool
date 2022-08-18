/****************************************************************************
** Meta object code from reading C++ file 'PointCloudGenerationTriEye.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.9)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../PointCloudGenerationTriEye.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PointCloudGenerationTriEye.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.9. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_PointCloudGenerationTriEye_t {
    QByteArrayData data[9];
    char stringdata0[230];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PointCloudGenerationTriEye_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PointCloudGenerationTriEye_t qt_meta_stringdata_PointCloudGenerationTriEye = {
    {
QT_MOC_LITERAL(0, 0, 26), // "PointCloudGenerationTriEye"
QT_MOC_LITERAL(1, 27, 34), // "selectCheckerboardImagesFolde..."
QT_MOC_LITERAL(2, 62, 0), // ""
QT_MOC_LITERAL(3, 63, 18), // "runCalibrationSlot"
QT_MOC_LITERAL(4, 82, 32), // "runDepthAndAggrDataSelectionSlot"
QT_MOC_LITERAL(5, 115, 27), // "runPointCloudGenerationSlot"
QT_MOC_LITERAL(6, 143, 24), // "closeDataSelectionWidget"
QT_MOC_LITERAL(7, 168, 31), // "runCalibrationDataSelectionSlot"
QT_MOC_LITERAL(8, 200, 29) // "closeCalibDataSelectionWidget"

    },
    "PointCloudGenerationTriEye\0"
    "selectCheckerboardImagesFolderSlot\0\0"
    "runCalibrationSlot\0runDepthAndAggrDataSelectionSlot\0"
    "runPointCloudGenerationSlot\0"
    "closeDataSelectionWidget\0"
    "runCalibrationDataSelectionSlot\0"
    "closeCalibDataSelectionWidget"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PointCloudGenerationTriEye[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x0a /* Public */,
       3,    0,   50,    2, 0x0a /* Public */,
       4,    0,   51,    2, 0x0a /* Public */,
       5,    0,   52,    2, 0x0a /* Public */,
       6,    0,   53,    2, 0x0a /* Public */,
       7,    0,   54,    2, 0x0a /* Public */,
       8,    0,   55,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void PointCloudGenerationTriEye::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PointCloudGenerationTriEye *_t = static_cast<PointCloudGenerationTriEye *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->selectCheckerboardImagesFolderSlot(); break;
        case 1: _t->runCalibrationSlot(); break;
        case 2: _t->runDepthAndAggrDataSelectionSlot(); break;
        case 3: _t->runPointCloudGenerationSlot(); break;
        case 4: _t->closeDataSelectionWidget(); break;
        case 5: _t->runCalibrationDataSelectionSlot(); break;
        case 6: _t->closeCalibDataSelectionWidget(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject PointCloudGenerationTriEye::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PointCloudGenerationTriEye.data,
      qt_meta_data_PointCloudGenerationTriEye,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *PointCloudGenerationTriEye::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PointCloudGenerationTriEye::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_PointCloudGenerationTriEye.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int PointCloudGenerationTriEye::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
