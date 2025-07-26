#include "RailJsonEditorDialog.h"
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSpinBox>
#include <array>

RailJsonEditorDialog::RailJsonEditorDialog(const QString& jsonString, QWidget* parent)
    : QDialog(parent), m_mainLayout(new QVBoxLayout(this)), m_formLayout(new QFormLayout())
{
    setWindowTitle("Edit Rail Properties");
    setMinimumWidth(400);
    // Parse JSON
    QJsonDocument doc = QJsonDocument::fromJson(jsonString.toUtf8());
    m_originalJson = doc.isObject() ? doc.object() : QJsonObject();
    buildFormFromJson(m_originalJson);
    m_mainLayout->addLayout(m_formLayout);
    // Buttons
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    m_saveButton = new QPushButton("Save");
    m_cancelButton = new QPushButton("Cancel");
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_saveButton);
    buttonLayout->addWidget(m_cancelButton);
    m_mainLayout->addLayout(buttonLayout);
    connect(m_saveButton, &QPushButton::clicked, this, &RailJsonEditorDialog::onSaveClicked);
    connect(m_cancelButton, &QPushButton::clicked, this, &RailJsonEditorDialog::onCancelClicked);
}

void RailJsonEditorDialog::buildFormFromJson(const QJsonObject& obj) {
    for (auto it = obj.begin(); it != obj.end(); ++it) {
        const QString& key = it.key();
        const QJsonValue& value = it.value();
        if (value.isDouble()) {
            QDoubleSpinBox* spin = new QDoubleSpinBox();
            spin->setDecimals(6);
            spin->setRange(-1000000.0, 1000000.0);
            spin->setSingleStep(0.01);
            spin->setValue(value.toDouble());
            m_formLayout->addRow(new QLabel(key), spin);
            m_fieldWidgets[key] = spin;
        } else if (value.isString()) {
            QLineEdit* edit = new QLineEdit(value.toString());
            m_formLayout->addRow(new QLabel(key), edit);
            m_fieldWidgets[key] = edit;
        } else if (value.isArray()) {
            QJsonArray arr = value.toArray();
            if (arr.size() == 3 && arr[0].isDouble() && arr[1].isDouble() && arr[2].isDouble()) {
                // 3-element vector
                QHBoxLayout* hbox = new QHBoxLayout();
                std::array<QSpinBox*, 3> spins;
                for (int i = 0; i < 3; ++i) {
                    spins[i] = new QSpinBox();
                    spins[i]->setRange(-1000000, 1000000);
                    spins[i]->setValue(static_cast<int>(arr[i].toDouble()));
                    hbox->addWidget(spins[i]);
                }
                QWidget* container = new QWidget();
                container->setLayout(hbox);
                m_formLayout->addRow(new QLabel(key), container);
                m_vectorFields[key] = spins;
            } else {
                // Fallback: show as string
                QLineEdit* edit = new QLineEdit(QString::fromUtf8(QJsonDocument(arr).toJson(QJsonDocument::Compact)));
                m_formLayout->addRow(new QLabel(key), edit);
                m_fieldWidgets[key] = edit;
            }
        } else {
            // Fallback: show as string
            QLineEdit* edit = new QLineEdit(value.toVariant().toString());
            m_formLayout->addRow(new QLabel(key), edit);
            m_fieldWidgets[key] = edit;
        }
    }
}

QJsonObject RailJsonEditorDialog::collectJsonFromForm() const {
    QJsonObject obj;
    // Scalars and strings
    for (auto it = m_fieldWidgets.begin(); it != m_fieldWidgets.end(); ++it) {
        const QString& key = it.key();
        QWidget* w = it.value();
        if (auto spin = qobject_cast<QDoubleSpinBox*>(w)) {
            obj[key] = spin->value();
        } else if (auto spinInt = qobject_cast<QSpinBox*>(w)) {
            obj[key] = spinInt->value();
        } else if (auto edit = qobject_cast<QLineEdit*>(w)) {
            obj[key] = edit->text();
        }
    }
    // 3-element vectors
    for (auto it = m_vectorFields.begin(); it != m_vectorFields.end(); ++it) {
        const QString& key = it.key();
        const auto& spins = it.value();
        QJsonArray arr;
        for (int i = 0; i < 3; ++i) arr.append(spins[i]->value());
        obj[key] = arr;
    }
    return obj;
}

QString RailJsonEditorDialog::getJsonString() const {
    QJsonObject obj = collectJsonFromForm();
    QJsonDocument doc(obj);
    return QString::fromUtf8(doc.toJson(QJsonDocument::Indented));
}

void RailJsonEditorDialog::onSaveClicked() {
    accept();
}

void RailJsonEditorDialog::onCancelClicked() {
    reject();
} 
