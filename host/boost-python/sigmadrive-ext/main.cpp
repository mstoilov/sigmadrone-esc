#include <iostream>
#include <boost/python.hpp>

#include "pidcontroller.h"

using PidControllerFloat = PidController<float>;

using namespace boost::python;

class World
{
public:
	World(std::string msg) :
			msg(msg)
	{
	}

	World(double a, double b)
	{
		msg = "double, double";
	}
	void set(std::string msg)
	{
		this->msg = msg;
	}

	std::string greet()
	{
		return msg;
	}

public:
	std::string msg;
};

struct Var
{
    Var(std::string name) : name(name), value() {}
    std::string const name;
    float value;
};

class Num
{
public:
	Num() {};
	float get() const			{ return val_; }
	void set(float value)		{ val_ = value; }

protected:
	float val_ = 0.0f;
};


struct Base
{
	virtual ~Base() {};
	virtual int f() { return 0; }
};

struct Derived : Base
{
	virtual int f() override { return 3; }
};


struct BaseWrap : Base, wrapper<Base>
{
    int f()
    {
    	override f = this->get_override("f");
        if (f)
            return f(); // *note*
        return Base::f();
    }

    int default_f() { return this->Base::f(); }
};

void b(Base*)
{

}

void d(Derived*)
{

}

Base* factory()
{
	return new Derived;
}

BOOST_PYTHON_MODULE(sigmadrive_ext)
{
    class_<PidControllerFloat>("PidController", init<float, float, float, float>())
        .add_property("Kp", &PidControllerFloat::GetKp, &PidControllerFloat::SetKp)
        .add_property("Ki", &PidControllerFloat::GetKi, &PidControllerFloat::SetKi)
        .add_property("Kd", &PidControllerFloat::GetKd, &PidControllerFloat::SetKd)
		.add_property("Kl", &PidControllerFloat::GetKl, &PidControllerFloat::SetKl)

    ;


    class_<World>("World", init<std::string>())
    	.def(init<double, double>())
        .def("greet", &World::greet)
        .def("set", &World::set)
    ;

    class_<Var>("Var", init<std::string>())
    	.def_readonly("name", &Var::name)
		.def_readwrite("value", &Var::value)
	;

    class_<Num>("Num")
    		.add_property("rovalue", &Num::get)
			.add_property("value", &Num::get, &Num::set)
	;

    def("b", b);
    def("d", d);

    // Tell Python to take ownership of factory's result
    def("factory", factory, return_value_policy<manage_new_object>());

    class_<BaseWrap, boost::noncopyable>("Base")
		.def("f", &Base::f, &BaseWrap::default_f)
        ;

    class_<Derived>("Derived");
}




