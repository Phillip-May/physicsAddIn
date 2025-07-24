#include "MaterialEditorDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QDebug>

MaterialEditorDialog::MaterialEditorDialog(QWidget* parent)
    : QDialog(parent)
    , m_materialColor(Qt::gray)
{
    setupUI();
    connectSignals();
    setWindowTitle("Add Custom Material");
}

MaterialEditorDialog::MaterialEditorDialog(const MaterialProperties& material, QWidget* parent)
    : QDialog(parent)
    , m_materialColor(material.color)
{
    setupUI();
    connectSignals();
    setMaterialProperties(material);
    setWindowTitle("Edit Material");
}

void MaterialEditorDialog::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    
    // Form layout for material properties
    QFormLayout* formLayout = new QFormLayout();
    
    // Name field
    m_nameEdit = new QLineEdit();
    m_nameEdit->setPlaceholderText("Enter material name");
    formLayout->addRow("Name:", m_nameEdit);
    
    // Static friction
    m_staticFrictionSpin = new QDoubleSpinBox();
    m_staticFrictionSpin->setRange(0.0, 2.0);
    m_staticFrictionSpin->setSingleStep(0.1);
    m_staticFrictionSpin->setValue(0.5);
    m_staticFrictionSpin->setSuffix(" (0-2)");
    formLayout->addRow("Static Friction:", m_staticFrictionSpin);
    
    // Dynamic friction
    m_dynamicFrictionSpin = new QDoubleSpinBox();
    m_dynamicFrictionSpin->setRange(0.0, 2.0);
    m_dynamicFrictionSpin->setSingleStep(0.1);
    m_dynamicFrictionSpin->setValue(0.3);
    m_dynamicFrictionSpin->setSuffix(" (0-2)");
    formLayout->addRow("Dynamic Friction:", m_dynamicFrictionSpin);
    
    // Restitution
    m_restitutionSpin = new QDoubleSpinBox();
    m_restitutionSpin->setRange(0.0, 1.0);
    m_restitutionSpin->setSingleStep(0.1);
    m_restitutionSpin->setValue(0.1);
    m_restitutionSpin->setSuffix(" (0-1)");
    formLayout->addRow("Restitution:", m_restitutionSpin);
    
    // Color button
    m_colorButton = new QPushButton();
    m_colorButton->setFixedSize(60, 30);
    m_colorButton->setStyleSheet(QString("background-color: %1; border: 1px solid black;").arg(m_materialColor.name()));
    formLayout->addRow("Color:", m_colorButton);
    
    // Description field
    m_descriptionEdit = new QLineEdit();
    m_descriptionEdit->setPlaceholderText("Enter material description (optional)");
    formLayout->addRow("Description:", m_descriptionEdit);
    
    mainLayout->addLayout(formLayout);
    
    // Buttons
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    
    m_saveButton = new QPushButton("Save");
    m_cancelButton = new QPushButton("Cancel");
    
    buttonLayout->addStretch();
    buttonLayout->addWidget(m_saveButton);
    buttonLayout->addWidget(m_cancelButton);
    
    mainLayout->addLayout(buttonLayout);
    
    setLayout(mainLayout);
    setFixedSize(400, 300);
}

void MaterialEditorDialog::connectSignals()
{
    connect(m_saveButton, &QPushButton::clicked, this, &MaterialEditorDialog::onSaveClicked);
    connect(m_cancelButton, &QPushButton::clicked, this, &MaterialEditorDialog::onCancelClicked);
    connect(m_colorButton, &QPushButton::clicked, this, &MaterialEditorDialog::onColorButtonClicked);
    
    // Connect validation signals
    connect(m_nameEdit, &QLineEdit::textChanged, this, &MaterialEditorDialog::validateInputs);
    connect(m_staticFrictionSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MaterialEditorDialog::validateInputs);
    connect(m_dynamicFrictionSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MaterialEditorDialog::validateInputs);
    connect(m_restitutionSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MaterialEditorDialog::validateInputs);
}

MaterialProperties MaterialEditorDialog::getMaterialProperties() const
{
    return MaterialProperties(
        m_nameEdit->text(),
        m_staticFrictionSpin->value(),
        m_dynamicFrictionSpin->value(),
        m_restitutionSpin->value(),
        m_materialColor,
        m_descriptionEdit->text()
    );
}

void MaterialEditorDialog::setMaterialProperties(const MaterialProperties& material)
{
    m_nameEdit->setText(material.name);
    m_staticFrictionSpin->setValue(material.staticFriction);
    m_dynamicFrictionSpin->setValue(material.dynamicFriction);
    m_restitutionSpin->setValue(material.restitution);
    m_descriptionEdit->setText(material.description);
    
    m_materialColor = material.color;
    m_colorButton->setStyleSheet(QString("background-color: %1; border: 1px solid black;").arg(m_materialColor.name()));
}

void MaterialEditorDialog::onSaveClicked()
{
    if (m_nameEdit->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, "Invalid Input", "Please enter a material name.");
        m_nameEdit->setFocus();
        return;
    }
    
    if (m_staticFrictionSpin->value() < 0.0 || m_staticFrictionSpin->value() > 2.0) {
        QMessageBox::warning(this, "Invalid Input", "Static friction must be between 0.0 and 2.0.");
        m_staticFrictionSpin->setFocus();
        return;
    }
    
    if (m_dynamicFrictionSpin->value() < 0.0 || m_dynamicFrictionSpin->value() > 2.0) {
        QMessageBox::warning(this, "Invalid Input", "Dynamic friction must be between 0.0 and 2.0.");
        m_dynamicFrictionSpin->setFocus();
        return;
    }
    
    if (m_restitutionSpin->value() < 0.0 || m_restitutionSpin->value() > 1.0) {
        QMessageBox::warning(this, "Invalid Input", "Restitution must be between 0.0 and 1.0.");
        m_restitutionSpin->setFocus();
        return;
    }
    
    accept();
}

void MaterialEditorDialog::onCancelClicked()
{
    reject();
}

void MaterialEditorDialog::onColorButtonClicked()
{
    QColor newColor = QColorDialog::getColor(m_materialColor, this, "Select Material Color");
    if (newColor.isValid()) {
        m_materialColor = newColor;
        m_colorButton->setStyleSheet(QString("background-color: %1; border: 1px solid black;").arg(m_materialColor.name()));
    }
}

void MaterialEditorDialog::validateInputs()
{
    bool isValid = !m_nameEdit->text().trimmed().isEmpty() &&
                   m_staticFrictionSpin->value() >= 0.0 && m_staticFrictionSpin->value() <= 2.0 &&
                   m_dynamicFrictionSpin->value() >= 0.0 && m_dynamicFrictionSpin->value() <= 2.0 &&
                   m_restitutionSpin->value() >= 0.0 && m_restitutionSpin->value() <= 1.0;
    
    m_saveButton->setEnabled(isValid);
} 