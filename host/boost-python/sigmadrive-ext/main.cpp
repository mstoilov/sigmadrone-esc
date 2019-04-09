#include <iostream>
#include <boost/python.hpp>
using namespace boost::python;

#include "pidcontroller.h"


using PidControllerFloat = PidController<float>;


BOOST_PYTHON_MODULE(sigmadrive_ext)
{
    class_<PidControllerFloat>("PidController")
    	.def(init<optional<float, float, float, float> >())
		.def("Input", &PidControllerFloat::Input)
		.def("Output", &PidControllerFloat::Output)
        .add_property("Kp", &PidControllerFloat::GetKp, &PidControllerFloat::SetKp)
        .add_property("Ki", &PidControllerFloat::GetKi, &PidControllerFloat::SetKi)
        .add_property("Kd", &PidControllerFloat::GetKd, &PidControllerFloat::SetKd)
		.add_property("Kl", &PidControllerFloat::GetKl, &PidControllerFloat::SetKl)
    ;

}




