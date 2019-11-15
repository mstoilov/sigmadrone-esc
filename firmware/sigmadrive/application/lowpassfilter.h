/*
 *  Sigmadrone
 *  Copyright (c) 2013-2015 The Sigmadrone Developers
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Martin Stoilov <martin@sigmadrone.org>
 *  Svetoslav Vassilev <svassilev@sigmadrone.org>
 */

#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_

template <typename DataType, typename CoeffType>
class LowPassFilter
{
public:
	LowPassFilter(CoeffType alpha) : alpha_(alpha) { Reset(); }
	LowPassFilter(CoeffType T, CoeffType RC) : alpha_(0) { SetAlpha(T, RC); Reset(); }
	~LowPassFilter() = default;

	void Reset(const DataType& out = {0})
	{
		out_ = out;
	}

	const DataType& DoFilter(const DataType& in)
	{
		out_ = out_ + (in - out_) * alpha_;
		return out_;
	}

	const DataType& Output() const
	{
		return out_;
	}

	void SetAlpha(const CoeffType& alpha)
	{
		alpha_ = alpha;
	}

	void SetAlpha(const CoeffType& T, const CoeffType& RC)
	{
		alpha_ = T / (T + RC);
	}

	CoeffType Alpha()
	{
		return alpha_;
	}

private:
	DataType out_;
	CoeffType alpha_;
};

#endif
