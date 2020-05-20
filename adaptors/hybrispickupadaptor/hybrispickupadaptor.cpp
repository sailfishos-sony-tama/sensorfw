/****************************************************************************
**
** Copyright (C) 2013 Jolla Ltd
** Contact: lorn.potter@jollamobile.com
**
** Copyright (C) 2020 Rinigus
** Contact: rinigus.git@gmail.com
**
**
** $QT_BEGIN_LICENSE:LGPL$
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "hybrispickupadaptor.h"
#include "logging.h"
#include "datatypes/utils.h"
#include "config.h"

#include <QtMath>

HybrisPickupAdaptor::HybrisPickupAdaptor(const QString& id) :
    HybrisAdaptor(id,SENSOR_TYPE_PICK_UP_GESTURE)
{
    buffer = new DeviceAdaptorRingBuffer<TapData>(1);
    setAdaptedSensor("hybrispickup", "Internal pickup triggers", buffer);

    setDescription("Hybris pickup");
    powerStatePath = SensorFrameworkConfig::configuration()->value("pickup/powerstate_path").toByteArray();
    if (!powerStatePath.isEmpty() && !QFile::exists(powerStatePath)) {
        sensordLogW() << "Path does not exists: " << powerStatePath;
        powerStatePath.clear();
    }
}

HybrisPickupAdaptor::~HybrisPickupAdaptor()
{
    delete buffer;
}

bool HybrisPickupAdaptor::startSensor()
{
    if (!(HybrisAdaptor::startSensor()))
        return false;
    if (isRunning() && !powerStatePath.isEmpty())
        writeToFile(powerStatePath, "1");
    sensordLogD() << "Hybris PickupAdaptor start\n";
    return true;
}

void HybrisPickupAdaptor::stopSensor()
{
    HybrisAdaptor::stopSensor();
    if (!isRunning() && !powerStatePath.isEmpty())
        writeToFile(powerStatePath, "0");
    sensordLogD() << "Hybris PickupAdaptor stop\n";
}

void HybrisPickupAdaptor::processSample(const sensors_event_t& data)
{
    // check if it is an expected event
#ifdef USE_BINDER
    const float value = data.u.data[0];
#else
    const float value = data.data[0];
#endif
    sensordLogD() << "Pickup value: " << value;
    if (qFabs(value - 1) > 1e-6) {
        sensordLogW() << "Unexpected pickup value: " << value;
        return;
    }
    
    TapData *d = buffer->nextSlot();
    d->timestamp_ = quint64(data.timestamp * .001);
    d->type_ = TapData::SingleTap;
    d->direction_ = TapData::Z;
    
    buffer->commit();
    buffer->wakeUpReaders();
}

void HybrisPickupAdaptor::restart()
{
    if (isRunning()) {
        stopSensor();
        startSensor();
    }
}

void HybrisPickupAdaptor::init()
{
}
