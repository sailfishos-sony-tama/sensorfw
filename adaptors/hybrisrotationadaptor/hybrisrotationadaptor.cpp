/****************************************************************************
**
** Copyright (C) 2013 Jolla Ltd
** Contact: lorn.potter@jollamobile.com
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

#include "hybrisrotationadaptor.h"
#include "logging.h"
#include "datatypes/utils.h"
#include "config.h"

#include <QtCore/qmath.h>

#define RADIANS_TO_DEGREES 57.2957795

/*
 * azimuth: angle between the magnetic north direction and the Y axis, around
 * the Z axis (0<=azimuth<360).
 * 0=North, 90=East, 180=South, 270=West
 *
 * pitch: Rotation around X axis (-180<=pitch<=180), with positive values when
 * the z-axis moves toward the y-axis.
 *
 * roll: Rotation around Y axis (-90<=roll<=90), with positive values when
 * the x-axis moves towards the z-axis.
 *
 **/

/*
 * not to be confused with rotation
 **/

HybrisRotationAdaptor::HybrisRotationAdaptor(const QString& id) :
    HybrisAdaptor(id,SENSOR_TYPE_ROTATION)
{
    buffer = new DeviceAdaptorRingBuffer<CompassData>(1);
    setAdaptedSensor("hybrisrotation", "Internal rotation coordinates", buffer);

    setDescription("Hybris rotation");
    powerStatePath = SensorFrameworkConfig::configuration()->value("rotation/powerstate_path").toByteArray();
    if (!powerStatePath.isEmpty() && !QFile::exists(powerStatePath))
    {
    	sensordLogW() << "Path does not exists: " << powerStatePath;
    	powerStatePath.clear();
    }
//    setDefaultInterval(50);
}

HybrisRotationAdaptor::~HybrisRotationAdaptor()
{
    delete buffer;
}

bool HybrisRotationAdaptor::startSensor()
{
    if (!(HybrisAdaptor::startSensor()))
        return false;
    if (isRunning() && !powerStatePath.isEmpty())
        writeToFile(powerStatePath, "1");
    sensordLogD() << "Hybris RotationAdaptor start\n";
    return true;
}

void HybrisRotationAdaptor::stopSensor()
{
    HybrisAdaptor::stopSensor();
    if (!isRunning() && !powerStatePath.isEmpty())
        writeToFile(powerStatePath, "0");
    sensordLogD() << "Hybris RotationAdaptor stop\n";
}

void HybrisRotationAdaptor::processSample(const sensors_event_t& data)
{
    CompassData *d = buffer->nextSlot();
    d->timestamp_ = quint64(data.timestamp * .001);
#ifdef USE_BINDER
#error "NOT IMPLEMENTED"
    // d->degrees_ = data.u.vec3.x; //azimuth
    // d->level_ = data.u.vec3.status;
#else
    float rotationVector[] = data.data;
#endif
    //
    float q0 = rotationVector[3];
    float q1 = rotationVector[0];
    float q2 = rotationVector[1];
    float q3 = rotationVector[2];
    
    float sq_q1 = 2 * q1 * q1;
    //float sq_q2 = 2 * q2 * q2;
    float sq_q3 = 2 * q3 * q3;
    float q1_q2 = 2 * q1 * q2;
    float q3_q0 = 2 * q3 * q0;
    //float q1_q3 = 2 * q1 * q3;
    //float q2_q0 = 2 * q2 * q0;
    //float q2_q3 = 2 * q2 * q3;
    //float q1_q0 = 2 * q1 * q0;

    float R1 = q1_q2 - q3_q0;
    float R4 = 1 - sq_q1 - sq_q3;
    azimuth = qAtan2(R1, R4) * RADIANS_TO_DEGREES;
    d->degrees_ = (int)(azimuth + 360) % 360;
    d->rawDegrees_ = d->degrees_;
    d->level_ = data.rotation.status;

    buffer->commit();
    buffer->wakeUpReaders();
}

void HybrisRotationAdaptor::init()
{
}
