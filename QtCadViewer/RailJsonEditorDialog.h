#ifndef RAILJSONEDITORDIALOG_H
#define RAILJSONEDITORDIALOG_H

#include <QDialog>
#include <QJsonObject>
#include <QJsonArray>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QMap>
#include <QVector3D>
#include <QJsonDocument>

class RailJsonEditorDialog : public QDialog {
    Q_OBJECT
public:
    explicit RailJsonEditorDialog(const QString& jsonString, QWidget* parent = nullptr);
    QString getJsonString() const;

private slots:
    void onSaveClicked();
    void onCancelClicked();

private:
    void buildFormFromJson(const QJsonObject& obj);
    QJsonObject collectJsonFromForm() const;

    QVBoxLayout* m_mainLayout;
    QFormLayout* m_formLayout;
    QPushButton* m_saveButton;
    QPushButton* m_cancelButton;
    QMap<QString, QWidget*> m_fieldWidgets;
    QMap<QString, std::array<QSpinBox*, 3>> m_vectorFields;
    QJsonObject m_originalJson;
};

#endif // RAILJSONEDITORDIALOG_H 