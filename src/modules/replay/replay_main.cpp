/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file replay_main.cpp
 * This module reads messages from an ULog file and publishes them.
 * It sets the parameters from the log file and handles user-defined
 * parameter overrides.
 *
 * @author Beat Kueng
*/

#include <drivers/drv_hrt.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include <cstring>
#include <float.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <logger/messages.h>

// for ekf2 replay
#include <uORB/topics/airspeed.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

#include "replay.hpp"

#define PARAMS_OVERRIDE_FILE PX4_ROOTFSDIR "/replay_params.txt"


extern "C" __EXPORT int replay_main(int argc, char *argv[]);

using namespace std;

namespace px4
{
class Replay;

namespace replay
{
Replay *instance = nullptr;
static int	control_task = -1;			//task handle for task

} //namespace replay


char *Replay::_replay_file = nullptr;

Replay::Replay()
{
}

Replay::~Replay()
{
	if (replay::control_task != -1) {
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned int i = 0;

		do {
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 200) {
				px4_task_delete(replay::control_task);
				replay::control_task = -1;
				break;
			}
		} while (replay::control_task != -1);
	}
}

void Replay::setupReplayFile(const char *file_name)
{
	if (_replay_file) {
		free(_replay_file);
	}

	_replay_file = strdup(file_name);
}



void Replay::setUserParams(const char *filename)
{
	string line, param_name, value_string;
	ifstream myfile(filename);

	if (!myfile.is_open()) {
		return;
	}

	PX4_INFO("Applying override params from %s...", filename);

	while (!myfile.eof()) {
		getline(myfile, line);

		if (line.empty() || line[0] == '#') {
			continue;
		}

		istringstream mystrstream(line);
		mystrstream >> param_name;
		mystrstream >> value_string;

		double param_value_double = stod(value_string);

		param_t handle = param_find(param_name.c_str());
		param_type_t param_format = param_type(handle);
		_overridden_params.insert(param_name);

		if (param_format == PARAM_TYPE_INT32) {
			int32_t value = 0;
			value = (int32_t)param_value_double;
			param_set(handle, (const void *)&value);

		} else if (param_format == PARAM_TYPE_FLOAT) {
			float value = 0;
			value = (float)param_value_double;
			param_set(handle, (const void *)&value);
		}
	}
}

bool Replay::readFileHeader(std::ifstream &file)
{
	file.seekg(0);
	ulog_file_header_s msg_header;
	file.read((char *)&msg_header, sizeof(msg_header));

	if (!file) {
		return false;
	}

	_file_start_time = msg_header.timestamp;
	//verify it's an ULog file
	char magic[8];
	magic[0] = 'U';
	magic[1] = 'L';
	magic[2] = 'o';
	magic[3] = 'g';
	magic[4] = 0x01;
	magic[5] = 0x12;
	magic[6] = 0x35;
	return memcmp(magic, msg_header.magic, 7) == 0;
}

bool Replay::readFileDefinitions(std::ifstream &file)
{
	PX4_INFO("Applying params from ULog file...");

	ulog_message_header_s message_header;
	file.seekg(sizeof(ulog_file_header_s));

	while (true) {
		file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

		if (!file) {
			return false;
		}

		switch (message_header.msg_type) {
		case (int)ULogMessageType::FORMAT:
			if (!readFormat(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::PARAMETER:
			if (!readAndApplyParameter(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::ADD_LOGGED_MSG:
			_data_section_start = file.tellg() - (streamoff)ULOG_MSG_HEADER_LEN;
			return true;

		case (int)ULogMessageType::INFO: //skip
			file.seekg(message_header.msg_size, ios::cur);
			break;

		default:
			PX4_ERR("unknown log definition type %i, size %i (offset %i)",
				(int)message_header.msg_type, (int)message_header.msg_size, (int)file.tellg());
			file.seekg(message_header.msg_size, ios::cur);
			break;
		}
	}

	return true;
}

bool Replay::readFormat(std::ifstream &file, uint16_t msg_size)
{
	_read_buffer.reserve(msg_size + 1);
	char *format = (char *)_read_buffer.data();
	file.read(format, msg_size);
	format[msg_size] = 0;

	if (!file) {
		return false;
	}

	string str_format(format);
	size_t pos = str_format.find(':');

	if (pos == string::npos) {
		return false;
	}

	string name = str_format.substr(0, pos);
	string fields = str_format.substr(pos + 1);
	_file_formats[name] = fields;

	return true;
}

bool Replay::readAndAddSubscription(std::ifstream &file, uint16_t msg_size)
{
	_read_buffer.reserve(msg_size + 1);
	char *message = (char *)_read_buffer.data();
	streampos this_message_pos = file.tellg() - (streamoff)ULOG_MSG_HEADER_LEN;
	file.read(message, msg_size);
	message[msg_size] = 0;

	if (!file) {
		return false;
	}

	if (file.tellg() <= _subscription_file_pos) { //already read this subscription
		return true;
	}

	_subscription_file_pos = file.tellg();

	uint8_t multi_id = *(uint8_t *)message;
	uint16_t msg_id = ((uint16_t) message[1]) | (((uint16_t) message[2]) << 8);
	string topic_name(message + 3);
	const orb_metadata *orb_meta = findTopic(topic_name);

	if (!orb_meta) {
		PX4_WARN("Topic %s not found internally. Will ignore it", topic_name.c_str());
		return true;
	}

	//check the format: the field definitions must match
	//FIXME: this should check recursively, all used nested types
	string file_format = _file_formats[topic_name];

	if (file_format != orb_meta->o_fields) {
		PX4_WARN("Formats for %s don't match. Will ignore it.", topic_name.c_str());
		PX4_WARN(" Internal format: %s", orb_meta->o_fields);
		PX4_WARN(" File format    : %s", file_format.c_str());
		return true; // not a fatal error
	}

	Subscription subscription;
	subscription.orb_meta = orb_meta;
	subscription.multi_id = multi_id;


	//find the timestamp offset (not necessarily the first field)
	string fields = orb_meta->o_fields;
	size_t prev_field_end = 0;
	size_t field_end = fields.find(';');
	bool timestamp_found = false;
	subscription.timestamp_offset = 0;

	while (field_end != string::npos && !timestamp_found) {
		size_t space_pos = fields.find(' ', prev_field_end);

		if (space_pos != string::npos) {
			string type_name_full = fields.substr(prev_field_end, space_pos - prev_field_end);
			string field_name = fields.substr(space_pos + 1, field_end - space_pos - 1);

			if (field_name == "timestamp") {
				timestamp_found = true;

				if (type_name_full != "uint64_t") {
					PX4_ERR("Unsupported timestamp type %s, ignoring the topic %s", type_name_full.c_str(),
						orb_meta->o_name);
					return true;
				}

			} else {
				subscription.timestamp_offset += sizeOfFullType(type_name_full);
			}
		}

		prev_field_end = field_end + 1;
		field_end = fields.find(';', prev_field_end);
	}

	if (!timestamp_found) {
		return true;
	}


	//find first data message (and the timestamp)
	streampos cur_pos = file.tellg();
	subscription.next_read_pos = this_message_pos; //this will be skipped

	if (!nextDataMessage(file, subscription, msg_id)) {
		return false;
	}

	file.seekg(cur_pos);

	if (!subscription.orb_meta) {
		//no message found. This is not a fatal error
		return true;
	}

	PX4_DEBUG("adding subscription for %s (msg_id %i)", subscription.orb_meta->o_name, msg_id);

	//add subscription
	if (_subscriptions.size() <= msg_id) {
		_subscriptions.resize(msg_id + 1);
	}

	_subscriptions[msg_id] = subscription;

	onSubscriptionAdded(_subscriptions[msg_id], msg_id);

	return true;
}

bool Replay::readAndHandleAdditionalMessages(std::ifstream &file, std::streampos end_position)
{
	ulog_message_header_s message_header;

	while (file.tellg() < end_position) {
		file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

		if (!file) {
			return false;
		}

		switch (message_header.msg_type) {
		case (int)ULogMessageType::PARAMETER:
			if (!readAndApplyParameter(file, message_header.msg_size)) {
				return false;
			}

			break;

		case (int)ULogMessageType::DROPOUT:
			readDropout(file, message_header.msg_size);
			break;

		default: //skip all others
			file.seekg(message_header.msg_size, ios::cur);
			break;
		}
	}

	return true;
}

bool Replay::readAndApplyParameter(std::ifstream &file, uint16_t msg_size)
{
	_read_buffer.reserve(msg_size);
	uint8_t *message = (uint8_t *)_read_buffer.data();
	file.read((char *)message, msg_size);

	if (!file) {
		return false;
	}

	uint8_t key_len = message[0];
	string key((char *)message + 1, key_len);

	size_t pos = key.find(' ');

	if (pos == string::npos) {
		return false;
	}

	string type = key.substr(0, pos);
	string param_name = key.substr(pos + 1);

	if (_overridden_params.find(param_name) != _overridden_params.end()) {
		//this parameter is overridden, so don't apply it
		return true;
	}

	if (type != "int32_t" && type != "float") {
		PX4_WARN("unknown parameter type %s, name %s (ignoring it)", type.c_str(), param_name.c_str());
		return true;
	}

	param_t handle = param_find(param_name.c_str());

	if (handle != PARAM_INVALID) {
		param_set(handle, (const void *)(message + 1 + key_len));
	}

	return true;
}

bool Replay::readDropout(std::ifstream &file, uint16_t msg_size)
{
	uint16_t duration;
	file.read((char *)&duration, sizeof(duration));

	PX4_INFO("Dropout in replayed log, %i ms", (int)duration);
	return file.good();
}

bool Replay::nextDataMessage(std::ifstream &file, Subscription &subscription, int msg_id)
{
	ulog_message_header_s message_header;
	file.seekg(subscription.next_read_pos);
	//ignore the first message (it's data we already read)
	file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

	if (file) {
		file.seekg(message_header.msg_size, ios::cur);
	}

	uint16_t file_msg_id;
	bool done = false;

	while (file && !done) {
		streampos cur_pos = file.tellg();
		file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

		if (!file) {
			break;
		}

		switch (message_header.msg_type) {
		case (int)ULogMessageType::ADD_LOGGED_MSG:
			readAndAddSubscription(file, message_header.msg_size);
			break;

		case (int)ULogMessageType::DATA:
			file.read((char *)&file_msg_id, sizeof(file_msg_id));

			if (file) {
				if (msg_id == file_msg_id) {
					if (message_header.msg_size == subscription.orb_meta->o_size_no_padding + 2) {
						subscription.next_read_pos = cur_pos;
						file.seekg(subscription.timestamp_offset, ios::cur);
						file.read((char *)&subscription.next_timestamp, sizeof(subscription.next_timestamp));
						done = true;

					} else { //sanity check failed!
						PX4_ERR("data message %s has wrong size %i (expected %i). Skipping",
							subscription.orb_meta->o_name, message_header.msg_size,
							subscription.orb_meta->o_size_no_padding + 2);
						file.seekg(message_header.msg_size - sizeof(file_msg_id), ios::cur);
					}

				} else { //not the one we are looking for
					file.seekg(message_header.msg_size - sizeof(file_msg_id), ios::cur);
				}
			}

			break;

		case (int)ULogMessageType::REMOVE_LOGGED_MSG: //skip these
		case (int)ULogMessageType::PARAMETER:
		case (int)ULogMessageType::DROPOUT:
		case (int)ULogMessageType::INFO:
		case (int)ULogMessageType::SYNC:
		case (int)ULogMessageType::LOGGING:
			file.seekg(message_header.msg_size, ios::cur);
			break;

		default:
			//this really should not happen
			PX4_ERR("unknown log message type %i, size %i (offset %i)",
				(int)message_header.msg_type, (int)message_header.msg_size, (int)file.tellg());
			file.seekg(message_header.msg_size, ios::cur);
			break;
		}
	}

	if (file.eof()) { //no more data messages for this subscription
		subscription.orb_meta = nullptr;
		file.clear();
	}

	return file.good();
}

const orb_metadata *Replay::findTopic(const std::string &name)
{
	const orb_metadata **topics = orb_get_topics();

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (name == topics[i]->o_name) {
			return topics[i];
		}
	}

	return nullptr;
}

std::string Replay::extractArraySize(const std::string &type_name_full, int &array_size)
{
	size_t start_pos = type_name_full.find('[');
	size_t end_pos = type_name_full.find(']');

	if (start_pos == string::npos || end_pos == string::npos) {
		array_size = 1;
		return type_name_full;
	}

	array_size = atoi(type_name_full.substr(start_pos + 1, end_pos - start_pos - 1).c_str());
	return type_name_full.substr(0, start_pos);
}

size_t Replay::sizeOfType(const std::string &type_name)
{
	if (type_name == "int8_t" || type_name == "uint8_t") {
		return 1;

	} else if (type_name == "int16_t" || type_name == "uint16_t") {
		return 2;

	} else if (type_name == "int32_t" || type_name == "uint32_t") {
		return 4;

	} else if (type_name == "int64_t" || type_name == "uint64_t") {
		return 8;

	} else if (type_name == "float") {
		return 4;

	} else if (type_name == "double") {
		return 8;

	} else if (type_name == "char" || type_name == "bool") {
		return 1;
	}

	const orb_metadata *orb_meta = findTopic(type_name);

	if (orb_meta) {
		return orb_meta->o_size;
	}

	PX4_ERR("unknown type: %s", type_name.c_str());
	return 0;
}
size_t Replay::sizeOfFullType(const std::string &type_name_full)
{
	int array_size;
	string type_name = extractArraySize(type_name_full, array_size);
	return sizeOfType(type_name) * array_size;
}

bool Replay::readDefinitionsAndApplyParams(std::ifstream &file)
{
	// log reader currently assumes little endian
	int num = 1;

	if (*(char *)&num != 1) {
		PX4_ERR("Replay only works on little endian!");
		return false;
	}

	if (!file.is_open()) {
		PX4_ERR("Failed to open replay file");
		return false;
	}

	if (!readFileHeader(file)) {
		PX4_ERR("Failed to read file header. Not a valid ULog file");
		return false;
	}

	//initialize the formats and apply the parameters from the log file
	if (!readFileDefinitions(file)) {
		PX4_ERR("Failed to read ULog definitions section. Broken file?");
		return false;
	}

	setUserParams(PARAMS_OVERRIDE_FILE);
	return true;
}

void Replay::task_main()
{
	ifstream replay_file(_replay_file, ios::in | ios::binary);

	if (!readDefinitionsAndApplyParams(replay_file)) {
		return;
	}

	onEnterMainLoop();

	_replay_start_time = hrt_absolute_time();

	PX4_INFO("Replay in progress...");

	ulog_message_header_s message_header;
	replay_file.seekg(_data_section_start);

	//we know the next message must be an ADD_LOGGED_MSG
	replay_file.read((char *)&message_header, ULOG_MSG_HEADER_LEN);

	if (!readAndAddSubscription(replay_file, message_header.msg_size)) {
		PX4_ERR("Failed to read subscription");
		return;
	}


	//we update the timestamps from the file by a constant offset to match
	//the current replay time
	const uint64_t timestamp_offset = _replay_start_time - _file_start_time;
	uint32_t nr_published_messages = 0;
	streampos last_additional_message_pos = _data_section_start;

	while (!_task_should_exit && replay_file) {

		//Find the next message to publish. Messages from different subscriptions don't need
		//to be in chronological order, so we need to check all subscriptions
		uint64_t next_file_time = 0;
		int next_msg_id = -1;

		for (size_t i = 0; i < _subscriptions.size(); ++i) {
			const Subscription &subscription = _subscriptions[i];

			if (subscription.orb_meta && !subscription.ignored) {
				if (next_file_time == 0 || subscription.next_timestamp < next_file_time) {
					next_msg_id = (int)i;
					next_file_time = subscription.next_timestamp;
				}
			}
		}

		if (next_msg_id == -1) {
			break; //no active subscription anymore. We're done.
		}

		Subscription &sub = _subscriptions[next_msg_id];

		if (next_file_time == 0) {
			//someone didn't set the timestamp properly. Consider the message invalid
			nextDataMessage(replay_file, sub, next_msg_id);
			continue;
		}


		//handle additional messages between last and next published data
		replay_file.seekg(last_additional_message_pos);
		streampos next_additional_message_pos = sub.next_read_pos;
		readAndHandleAdditionalMessages(replay_file, next_additional_message_pos);
		last_additional_message_pos = next_additional_message_pos;


		const uint64_t publish_timestamp = handleTopicDelay(next_file_time, timestamp_offset);


		//It's time to publish
		readTopicDataToBuffer(sub, replay_file);
		memcpy(_read_buffer.data() + sub.timestamp_offset, &publish_timestamp, sizeof(uint64_t)); //adjust the timestamp

		if (handleTopicUpdate(sub, _read_buffer.data(), replay_file)) {
			++nr_published_messages;
		}

		nextDataMessage(replay_file, sub, next_msg_id);

		//TODO: output status (eg. every sec), including total duration...
	}

	for (auto &subscription : _subscriptions) {
		if (subscription.orb_advert) {
			orb_unadvertise(subscription.orb_advert);
			subscription.orb_advert = nullptr;
		}
	}

	if (!_task_should_exit) {
		PX4_INFO("Replay done (published %u msgs, %.3lf s)", nr_published_messages,
			 (double)hrt_elapsed_time(&_replay_start_time) / 1.e6);

		//TODO: should we close the log file & exit (optionally, by adding a parameter -q) ?
	}

	onExitMainLoop();
}

void Replay::readTopicDataToBuffer(const Subscription &sub, std::ifstream &replay_file)
{
	const size_t msg_read_size = sub.orb_meta->o_size_no_padding;
	const size_t msg_write_size = sub.orb_meta->o_size;
	_read_buffer.reserve(msg_write_size);
	replay_file.seekg(sub.next_read_pos + (streamoff)(ULOG_MSG_HEADER_LEN + 2)); //skip header & msg id
	replay_file.read((char *)_read_buffer.data(), msg_read_size);
}

bool Replay::handleTopicUpdate(Subscription &sub, void *data, std::ifstream &replay_file)
{
	return publishTopic(sub, data);
}

uint64_t Replay::handleTopicDelay(uint64_t next_file_time, uint64_t timestamp_offset)
{

	const uint64_t publish_timestamp = next_file_time + timestamp_offset;

	//wait if necessary
	uint64_t cur_time = hrt_absolute_time();

	// if some topics have a timestamp smaller than the log file start, publish them immediately
	if (cur_time < publish_timestamp && next_file_time > _file_start_time) {
		usleep(publish_timestamp - cur_time);
	}

	return publish_timestamp;
}

bool Replay::publishTopic(Subscription &sub, void *data)
{
	bool published = false;

	if (sub.orb_advert) {
		orb_publish(sub.orb_meta, sub.orb_advert, data);
		published = true;

	} else {
		if (sub.multi_id == 0) {
			sub.orb_advert = orb_advertise(sub.orb_meta, data);
			published = true;

		} else {
			// make sure the other instances are advertised already so that we get the correct instance
			bool advertised = false;

			for (const auto &subscription : _subscriptions) {
				if (subscription.orb_meta) {
					if (strcmp(sub.orb_meta->o_name, subscription.orb_meta->o_name) == 0 &&
					    subscription.orb_advert && subscription.multi_id == sub.multi_id - 1) {
						advertised = true;
					}
				}
			}

			if (advertised) {
				int instance;
				sub.orb_advert = orb_advertise_multi(sub.orb_meta, data, &instance, ORB_PRIO_DEFAULT);
				published = true;
			}
		}
	}

	if (published) {
		++sub.publication_counter;
	}

	return published;
}

bool ReplayEkf2::handleTopicUpdate(Subscription &sub, void *data, std::ifstream &replay_file)
{
	if (sub.orb_meta == ORB_ID(ekf2_timestamps)) {
		ekf2_timestamps_s ekf2_timestamps;
		memcpy(&ekf2_timestamps, data, sub.orb_meta->o_size);

		if (!publishEkf2Topics(ekf2_timestamps, replay_file)) {
			return false;
		}

		px4_pollfd_struct_t fds[1];
		fds[0].fd = _vehicle_attitude_sub;
		fds[0].events = POLLIN;
		// wait for a response from the estimator
		int pret = px4_poll(fds, 1, 1000);

		// introduce some breaks to make sure the logger can keep up
		if (++_topic_counter == 50) {
			usleep(1000);
			_topic_counter = 0;
		}

		if (pret == 0) {
			PX4_WARN("poll timeout");

		} else if (pret < 0) {
			PX4_ERR("poll failed (%i)", pret);

		} else {
			if (fds[0].revents & POLLIN) {
				vehicle_attitude_s att;
				// need to to an orb_copy so that poll will not return immediately
				orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &att);
			}
		}

		return true;

	} else if (sub.orb_meta == ORB_ID(vehicle_status) || sub.orb_meta == ORB_ID(vehicle_land_detected)) {
		return publishTopic(sub, data);
	} // else: do not publish

	return false;
}

void ReplayEkf2::onSubscriptionAdded(Subscription &sub, uint16_t msg_id)
{
	if (sub.orb_meta == ORB_ID(sensor_combined)) {
		_sensors_combined_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(vehicle_gps_position)) {
		_gps_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(optical_flow)) {
		_optical_flow_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(distance_sensor)) {
		_distance_sensor_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(airspeed)) {
		_airspeed_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(vehicle_vision_position)) {
		_vehicle_vision_position_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(vehicle_vision_attitude)) {
		_vehicle_vision_attitude_msg_id = msg_id;
	}

	// the main loop should only handle publication of the following topics, the sensor topics are
	// handled separately in publishEkf2Topics()
	sub.ignored = sub.orb_meta != ORB_ID(ekf2_timestamps) && sub.orb_meta != ORB_ID(vehicle_status)
		      && sub.orb_meta != ORB_ID(vehicle_land_detected);
}

bool ReplayEkf2::publishEkf2Topics(const ekf2_timestamps_s &ekf2_timestamps, std::ifstream &replay_file)
{
	auto handle_sensor_publication = [&](int16_t timestamp_relative, uint16_t msg_id) {
		if (timestamp_relative != ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID) {
			// timestamp_relative is already given in 0.1 ms
			uint64_t t = timestamp_relative + ekf2_timestamps.timestamp / 100; // in 0.1 ms
			findTimestampAndPublish(t, msg_id, replay_file);
		}
	};
	handle_sensor_publication(ekf2_timestamps.gps_timestamp_rel, _gps_msg_id); // gps
	handle_sensor_publication(ekf2_timestamps.optical_flow_timestamp_rel, _optical_flow_msg_id); // optical flow
	handle_sensor_publication(ekf2_timestamps.distance_sensor_timestamp_rel, _distance_sensor_msg_id); // distance sensor
	handle_sensor_publication(ekf2_timestamps.airspeed_timestamp_rel, _airspeed_msg_id); // airspeed
	handle_sensor_publication(ekf2_timestamps.vision_position_timestamp_rel,
				  _vehicle_vision_position_msg_id); // vision position
	handle_sensor_publication(ekf2_timestamps.vision_attitude_timestamp_rel,
				  _vehicle_vision_attitude_msg_id); // vision attitude

	// sensor_combined: publish last because ekf2 is polling on this
	if (!findTimestampAndPublish(ekf2_timestamps.timestamp / 100, _sensors_combined_msg_id, replay_file)) {
		if (_sensors_combined_msg_id == msg_id_invalid) {
			// subscription not found yet or sensor_combined not contained in log
			return false;

		} else if (!_subscriptions[_sensors_combined_msg_id].orb_meta) {
			return false; // read past end of file

		} else {
			// we should publish a topic, just publish the same again
			readTopicDataToBuffer(_subscriptions[_sensors_combined_msg_id], replay_file);
			publishTopic(_subscriptions[_sensors_combined_msg_id], _read_buffer.data());
		}
	}

	return true;

}

bool ReplayEkf2::findTimestampAndPublish(uint64_t timestamp, uint16_t msg_id, std::ifstream &replay_file)
{
	if (msg_id == msg_id_invalid) {
		// could happen if a topic is not logged
		return false;
	}

	Subscription &sub = _subscriptions[msg_id];

	while (sub.next_timestamp / 100 < timestamp && sub.orb_meta) {
		nextDataMessage(replay_file, sub, msg_id);
	}

	if (!sub.orb_meta) { // no messages anymore
		return false;
	}

	if (sub.next_timestamp / 100 != timestamp) {
		// this can happen in beginning of the log or on a dropout
		PX4_DEBUG("No timestamp match found for topic %s (%i, %i)", sub.orb_meta->o_name, (int)sub.next_timestamp / 100,
			  timestamp);
		++sub.error_counter;
		return false;
	}

	readTopicDataToBuffer(sub, replay_file);
	publishTopic(sub, _read_buffer.data());
	return true;
}

void ReplayEkf2::onEnterMainLoop()
{
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
}

void ReplayEkf2::onExitMainLoop()
{
	// print statistics
	auto print_sensor_statistics = [this](uint16_t msg_id, const char *name) {
		if (msg_id != msg_id_invalid) {
			Subscription &sub = _subscriptions[msg_id];

			if (sub.publication_counter > 0 || sub.error_counter > 0) {
				PX4_INFO("%s: %i, %i", name, sub.publication_counter, sub.error_counter);
			}
		}
	};

	PX4_INFO("");
	PX4_INFO("Topic, Num Published, Num Error (no timestamp match found):");
	print_sensor_statistics(_sensors_combined_msg_id, "sensor_combined");
	print_sensor_statistics(_gps_msg_id, "vehicle_gps_position");
	print_sensor_statistics(_optical_flow_msg_id, "optical_flow");
	print_sensor_statistics(_distance_sensor_msg_id, "distance_sensor");
	print_sensor_statistics(_airspeed_msg_id, "airspeed");
	print_sensor_statistics(_vehicle_vision_position_msg_id, "vehicle_vision_position");
	print_sensor_statistics(_vehicle_vision_attitude_msg_id, "vehicle_vision_attitude");

	orb_unsubscribe(_vehicle_attitude_sub);
	_vehicle_attitude_sub = -1;
}

uint64_t ReplayEkf2::handleTopicDelay(uint64_t next_file_time, uint64_t timestamp_offset)
{
	// no need for usleep
	return next_file_time;
}

void Replay::task_main_trampoline(int argc, char *argv[])
{
	// check the replay mode
	const char *replay_mode = getenv(replay::ENV_MODE);

	if (replay_mode && strcmp(replay_mode, "ekf2") == 0) {
		PX4_INFO("Ekf2 replay mode");
		replay::instance = new ReplayEkf2();

	} else {
		replay::instance = new Replay();
	}

	if (replay::instance == nullptr) {
		PX4_ERR("alloc failed");
		return;
	}

	replay::instance->task_main();
	replay::control_task = -1;
}

int Replay::start(bool quiet, bool apply_params_only)
{
	ASSERT(replay::control_task == -1);
	int ret = PX4_OK;

	//check for logfile env variable
	const char *logfile = getenv(replay::ENV_FILENAME);

	if (logfile) {
		if (!isSetup()) {
			PX4_INFO("using replay log file: %s", logfile);
			setupReplayFile(logfile);
		}

	} else {
		if (quiet) {
			return PX4_OK;

		} else {
			PX4_ERR("no log file given (via env variable %s)", replay::ENV_FILENAME);
			return -1;
		}
	}

	if (apply_params_only) {
		Replay *r = new Replay();

		if (r == nullptr) {
			PX4_ERR("alloc failed");
			return -ENOMEM;
		}

		ifstream replay_file(_replay_file, ios::in | ios::binary);

		if (!r->readDefinitionsAndApplyParams(replay_file)) {
			ret = -1;
		}

		delete (r);

	} else {

		/* start the task */
		replay::control_task = px4_task_spawn_cmd("replay",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       4000,
				       (px4_main_t)&Replay::task_main_trampoline,
				       nullptr);

		if (replay::control_task < 0) {
			replay::control_task = -1;
			PX4_ERR("task start failed");
			return -errno;
		}
	}

	return ret;
}

} //namespace px4

using namespace px4;

int replay_main(int argc, char *argv[])
{
	if (argc < 1) {
		PX4_WARN("usage: replay {tryapplyparams|trystart|start|stop|status}");
		return 1;
	}

	bool do_start = false;
	bool quiet = false;
	bool apply_params_only = false;

	if (!strcmp(argv[1], "start")) {
		do_start = true;

	} else if (!strcmp(argv[1], "trystart")) {
		do_start = true;
		quiet = true;

	} else if (!strcmp(argv[1], "tryapplyparams")) {
		do_start = true;
		quiet = true;
		apply_params_only = true;
	}

	if (do_start) {
		if (replay::instance != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		if (PX4_OK != Replay::start(quiet, apply_params_only)) {
			PX4_ERR("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (replay::instance == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

		delete replay::instance;
		replay::instance = nullptr;

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (replay::instance) {
			PX4_WARN("running");
			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

	PX4_ERR("unrecognized command");
	return 1;
}
