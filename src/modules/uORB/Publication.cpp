/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Publication.cpp
 *
 */

#include "Publication.hpp"
#include "topics/actuator_controls.h"
#include "topics/actuator_direct.h"
#include "topics/actuator_outputs.h"
#include "topics/debug_key_value.h"
#include "topics/ekf2_innovations.h"
#include "topics/filtered_bottom_flow.h"
#include "topics/rc_channels.h"
#include "topics/tecs_status.h"
#include "topics/vehicle_attitude.h"
#include "topics/vehicle_attitude_setpoint.h"
#include "topics/vehicle_global_position.h"
#include "topics/vehicle_global_velocity_setpoint.h"
#include "topics/vehicle_local_position.h"
#include "topics/vehicle_rates_setpoint.h"

#include <px4_defines.h>

namespace uORB
{

PublicationBase::PublicationBase(const struct orb_metadata *meta,
				 int priority) :
	_meta(meta),
	_priority(priority),
	_instance(),
	_handle(nullptr)
{
}

void PublicationBase::update(void *data)
{
	if (_handle != nullptr) {
		int ret = orb_publish(getMeta(), getHandle(), data);

		if (ret != PX4_OK) { warnx("publish fail"); }

	} else {
		orb_advert_t handle;

		if (_priority > 0) {
			handle = orb_advertise_multi(
					 getMeta(), data,
					 &_instance, _priority);

		} else {
			handle = orb_advertise(getMeta(), data);
		}

		if (int64_t(handle) != PX4_ERROR) {
			setHandle(handle);

		} else {
			warnx("advert fail");
		}
	}
}

PublicationBase::~PublicationBase()
{
	orb_unadvertise(getHandle());
}

PublicationNode::PublicationNode(const struct orb_metadata *meta,
				 int priority,
				 List<PublicationNode *> *list) :
	PublicationBase(meta, priority)
{
	if (list != nullptr) { list->add(this); }
}

// explicit template instantiation
template class __EXPORT Publication<actuator_controls_s>;
template class __EXPORT Publication<actuator_direct_s>;
template class __EXPORT Publication<actuator_outputs_s>;
template class __EXPORT Publication<debug_key_value_s>;
template class __EXPORT Publication<ekf2_innovations_s>;
template class __EXPORT Publication<filtered_bottom_flow_s>;
template class __EXPORT Publication<rc_channels_s>;
template class __EXPORT Publication<tecs_status_s>;
template class __EXPORT Publication<vehicle_attitude_s>;
template class __EXPORT Publication<vehicle_attitude_setpoint_s>;
template class __EXPORT Publication<vehicle_global_position_s>;
template class __EXPORT Publication<vehicle_global_velocity_setpoint_s>;
template class __EXPORT Publication<vehicle_local_position_s>;
template class __EXPORT Publication<vehicle_rates_setpoint_s>;

}
