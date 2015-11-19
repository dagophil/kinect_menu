#ifndef KINECT_HXX
#define KINECT_HXX

#include <stdexcept>
#include <array>
#include <algorithm>
#include <numeric>

#include <ni/XnOS.h>
#include <ni/XnCppWrapper.h>

#include "ndarray.hxx"

class Kinect{

public:

    Kinect()
    {
        check_error(context_.Init());
        check_error(depth_.Create(context_));
        check_error(context_.StartGeneratingAll());
        depth_.GetMetaData(depth_meta_);
        depth_data_.resize(depth_meta_.XRes(), depth_meta_.YRes());
        depth_rgba_.resize(depth_meta_.XRes(), depth_meta_.YRes());
    }

    ~Kinect()
    {
        context_.Release();
    }

    Kinect(Kinect const & other) = delete;
    Kinect & operator=(Kinect const & other) = delete;

    XnUInt32 XRes() const
    {
        return depth_meta_.XRes();
    }

    XnUInt32 YRes() const
    {
        return depth_meta_.YRes();
    }

    XnUInt32 ZRes() const
    {
        return depth_meta_.ZRes();
    }

    void wait_for_update()
    {
        check_error(context_.WaitAnyUpdateAll());
        depth_.GetMetaData(depth_meta_);
        for (size_t y = 0; y < YRes(); ++y)
        {
            for (size_t x = 0; x < XRes(); ++x)
            {
                depth_data_(x, y) = depth_meta_(x, y);
            }
        }
        create_histogram();
    }

    Array2D<std::array<uint8_t, 4> > const & depth_rgba() const
    {
        return depth_rgba_;
    }

    uint8_t const * depth_rgba_ptr() const
    {
        return &(depth_rgba_.front()[0]);
    }

private:

    void check_error(XnStatus status){
        if (status != XN_STATUS_OK){
            throw std::runtime_error("Kinect error: " + std::string(xnGetStatusString(status)));
        }
    }

    void create_histogram()
    {
        std::vector<float> hist(ZRes(), 0.0f);
        size_t num_points = 0;

        for (size_t y = 0; y < depth_data_.height(); ++y)
        {
            for (size_t x = 0; x < depth_data_.width(); ++x)
            {
                if (depth_data_(x, y) != 0)
                {
                    ++hist[depth_data_(x, y)];
                    ++num_points;
                }
            }
        }
        for (size_t i = 1; i < hist.size(); ++i)
        {
            hist[i] += hist[i-1];
        }
        if (num_points > 0)
        {
            for (size_t i = 0; i < hist.size(); ++i)
            {
                hist[i] = (unsigned int) (256 * (1.0f - hist[i] / num_points));
            }
        }
        for (size_t y = 0; y < depth_rgba_.height(); ++y)
        {
            for (size_t x = 0; x < depth_rgba_.width(); ++x)
            {
                auto const v = hist[depth_data_(x, y)];
                depth_rgba_(x, y)[0] = v;
                depth_rgba_(x, y)[1] = v;
                depth_rgba_(x, y)[2] = 0;
                depth_rgba_(x, y)[3] = 255;
            }
        }
    }

    xn::Context context_;
    xn::DepthGenerator depth_;
    xn::DepthMetaData depth_meta_;

    Array2D<XnDepthPixel> depth_data_;
    Array2D<std::array<uint8_t, 4> > depth_rgba_;

};


#endif
