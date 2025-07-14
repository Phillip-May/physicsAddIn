#ifndef OBJECTPROPERTIESDIALOG_H
#define OBJECTPROPERTIESDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QFormLayout>
#include <QLabel>
#include <QCheckBox>
#include <QComboBox>
#include "ObjectPropertiesManager.h"

class ObjectPropertiesDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ObjectPropertiesDialog(Item item, ObjectPropertiesManager* manager, MaterialManager* materialManager = nullptr, QWidget* parent = nullptr);
    
    ObjectPropertiesManager::ObjectProperties getObjectProperties() const;
    void setObjectProperties(const ObjectPropertiesManager::ObjectProperties& properties);

private slots:
    void onSaveClicked();
    void onCancelClicked();
    void onUseCustomPropertiesToggled(bool checked);
    void onUseCustomCenterOfMassToggled(bool checked);
    void validateInputs();

private:
    void setupUI();
    void connectSignals();
    void updateUI();
    
    Item m_item;
    ObjectPropertiesManager* m_manager;
    MaterialManager* m_materialManager;
    
    // UI elements
    QCheckBox* m_useCustomPropertiesCheck;
    QCheckBox* m_useCustomCenterOfMassCheck;
    QDoubleSpinBox* m_massSpin;
    QDoubleSpinBox* m_centerOfMassXSpin;
    QDoubleSpinBox* m_centerOfMassYSpin;
    QDoubleSpinBox* m_centerOfMassZSpin;
    QComboBox* m_materialCombo;
    QPushButton* m_saveButton;
    QPushButton* m_cancelButton;
    
    ObjectPropertiesManager::ObjectProperties m_currentProperties;
};

#endif // OBJECTPROPERTIESDIALOG_H 