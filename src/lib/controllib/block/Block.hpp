/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file Block.h
 *
 * Controller library code
 */

#pragma once

#include <stdint.h>
#include <inttypes.h>

#include <containers/List.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <controllib/block/BlockParam.hpp>

namespace control
{

static const uint16_t maxChildrenPerBlock = 100;
static const uint16_t maxParamsPerBlock = 100;
static const uint16_t maxSubscriptionsPerBlock = 100;
static const uint16_t maxPublicationsPerBlock = 100;
static const uint8_t blockNameLengthMax = 40;

// forward declaration
class BlockParamBase;
class SuperBlock;

/**
 */
class __EXPORT Block :
	public ListNode<Block *>
{
public:
	friend class BlockParamBase;
// methods
	Block(SuperBlock *parent, const char *name);
	void getName(char *name, size_t n);
	virtual ~Block() {};
	virtual void updateParams();
	virtual void updateSubscriptions();
	virtual void updatePublications();
	virtual void setDt(float dt) { _dt = dt; }
// accessors
	float getDt() { return _dt; }
protected:
// accessors
	SuperBlock *getParent() { return _parent; }
	List<uORB::SubscriptionNode *> &getSubscriptions() { return _subscriptions; }
	List<uORB::PublicationNode *> &getPublications() { return _publications; }
	List<BlockParamBase *> &getParams() { return _params; }
// attributes
	const char *_name;
	SuperBlock *_parent;
	float _dt;
	List<uORB::SubscriptionNode *> _subscriptions;
	List<uORB::PublicationNode *> _publications;
	List<BlockParamBase *> _params;

private:
	/* this class has pointer data members and should not be copied (private constructor) */
	Block(const control::Block &);
	Block operator=(const control::Block &);
};

class __EXPORT SuperBlock :
	public Block
{
public:
	friend class Block;
// methods
	SuperBlock(SuperBlock *parent, const char *name) :
		Block(parent, name),
		_children()
	{
	}
	virtual ~SuperBlock() {};
	virtual void setDt(float dt);
	virtual void updateParams()
	{
		Block::updateParams();

		if (getChildren().getHead() != NULL) { updateChildParams(); }
	}
	virtual void updateSubscriptions()
	{
		Block::updateSubscriptions();

		if (getChildren().getHead() != NULL) { updateChildSubscriptions(); }
	}
	virtual void updatePublications()
	{
		Block::updatePublications();

		if (getChildren().getHead() != NULL) { updateChildPublications(); }
	}
protected:
// methods
	List<Block *> &getChildren() { return _children; }
	void updateChildParams();
	void updateChildSubscriptions();
	void updateChildPublications();
// attributes
	List<Block *> _children;
};


} // namespace control
