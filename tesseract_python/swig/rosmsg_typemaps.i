%{
#include <boost/scope_exit.hpp>
#include <boost/algorithm/string.hpp>
%}

%fragment("rosmsg_fragments", "header")
{
	template <class T>
	bool ConvertFromPythonMsgToCppMsg(T& cpp_msg, PyObject* py_msg)
	{
		PyObject* sys_mod_dict = PyImport_GetModuleDict();
		if (!sys_mod_dict) return false;
		PyObject* io_mod = PyMapping_GetItemString(sys_mod_dict, "io");
		if (!io_mod) return false;
		PyObject* bytes_io = PyObject_CallMethod(io_mod, "BytesIO", "");
		if (!bytes_io) return false;
		BOOST_SCOPE_EXIT_TPL(&bytes_io) {		
			Py_XDECREF(bytes_io);
		} BOOST_SCOPE_EXIT_END
		PyObject* serialize_res = PyObject_CallMethod(py_msg, "serialize", "(O)", bytes_io);
		if (!serialize_res) return false;
		Py_XDECREF(serialize_res);
		if (PyErr_Occurred()) return false;		
		PyObject* msg_bytes = PyObject_CallMethod(bytes_io,"getvalue", NULL);
		if (!msg_bytes) return false;
		BOOST_SCOPE_EXIT_TPL(&msg_bytes) {
			Py_XDECREF(msg_bytes);
		} BOOST_SCOPE_EXIT_END
		Py_buffer msg_bytes_buffer;
		if (!PyObject_CheckBuffer(msg_bytes)) return false;
		if (PyObject_GetBuffer(msg_bytes,&msg_bytes_buffer,PyBUF_SIMPLE)<0) return false;
		BOOST_SCOPE_EXIT_TPL(&msg_bytes_buffer) {
			PyBuffer_Release(&msg_bytes_buffer);
		} BOOST_SCOPE_EXIT_END
		ros::serialization::IStream ros_istream((uint8_t*)msg_bytes_buffer.buf, msg_bytes_buffer.len);
		ros::serialization::deserialize(ros_istream, cpp_msg);
		return true;
	}
	
	template<class T>
	bool ConvertFromCppMsgToPythonMsg(PyObject** out, const T* cpp_msg)
	{
		uint32_t serial_size = ros::serialization::serializationLength(*cpp_msg);
		boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

		ros::serialization::OStream stream(buffer.get(), serial_size);
		ros::serialization::serialize(stream, *cpp_msg);
		
		PyObject* buffer_str=PyString_FromStringAndSize((char*)buffer.get(), serial_size);
		if (!buffer_str) return false;
		BOOST_SCOPE_EXIT_TPL(&buffer_str) {
			Py_XDECREF(buffer_str);
		} BOOST_SCOPE_EXIT_END
		PyObject* sys_mod_dict = PyImport_GetModuleDict();
		if (!sys_mod_dict) return false;
				
		std::string msg_name(ros::message_traits::DataType<T>::value());
		std::vector<std::string> msg_name_split;
		boost::split(msg_name_split,msg_name, boost::is_any_of("/"));
		if (msg_name_split.size()!=2) return false;
		std::string msg_module_name = msg_name_split[0] + ".msg";
		PyObject* msg_module;
		if (PyMapping_HasKeyString(sys_mod_dict, &msg_module_name[0]))
		{
			msg_module = PyMapping_GetItemString(sys_mod_dict, &msg_module_name[0]);
			Py_XINCREF(msg_module);
					
		}
		else
		{
			msg_module = PyImport_ImportModule(&msg_module_name[0]);
			
		}
		
		if (!msg_module) return false;
		BOOST_SCOPE_EXIT_TPL(&msg_module) {		
			Py_XDECREF(msg_module);
		} BOOST_SCOPE_EXIT_END
		
		PyObject* py_msg = PyObject_CallMethod(msg_module, &msg_name_split[1][0], "");
		if (!py_msg) return false;		
		BOOST_SCOPE_EXIT_TPL(&py_msg) {		
			Py_XDECREF(py_msg);
		} BOOST_SCOPE_EXIT_END
				
		PyObject* deserialize_res = PyObject_CallMethod(py_msg, "deserialize", "(O)", buffer_str);
		if (!deserialize_res) return false;
		Py_XDECREF(deserialize_res);
		
		*out=py_msg;
		Py_XINCREF(py_msg);
		return true;
	}
}
	
%define %rosmsg_typemaps(CLASS)

%typemap(in, fragment="rosmsg_fragments") CLASS (CLASS temp)
{
	if(!ConvertFromPythonMsgToCppMsg(temp, $input))
		SWIG_fail;
	$1 = temp;	
}

%typemap(in, fragment="rosmsg_fragments") CLASS const & (CLASS temp)
{
	if(!ConvertFromPythonMsgToCppMsg(temp, $input))
		SWIG_fail;
	$1 = &temp;	
}

%typemap(argout, fragment="rosmsg_fragments") CLASS const & (PyObject* temp) ""

%typemap(argout, fragment="rosmsg_fragments") CLASS & (PyObject* temp)
{
  // Argout: &  
  if (!ConvertFromCppMsgToPythonMsg<CLASS >(&temp, $1))
    SWIG_fail;
  $result=temp;
}

%typemap(in, fragment="rosmsg_fragments", numinputs=0) CLASS & (CLASS temp)
{
	$1 = &temp;
}

		
%enddef