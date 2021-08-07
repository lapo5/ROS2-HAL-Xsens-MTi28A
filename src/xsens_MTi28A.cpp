#include "libcmt/cmt2.h"
#include <cstdlib>
#include <cstdio>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <string>
#include <exception>

using namespace xsens;
namespace py = pybind11;

enum Mode {
	CalibratedData = 0,
	OrientationEstimate = 1,
	All = 2
};

class PyHALXsens{
	private:
		Cmt2s serial;
		Message msg, reply;
		int mode;
	public:
		PyHALXsens(){}

		bool init(int mode, std::string port){
		    if (serial.open(port.c_str(), B115200) != XRV_OK){
		    	std::cout << "Failed to open " << port << std::endl;
		    	return false;
		    }
			msg.setMessageId(CMT_MID_GOTOCONFIG);
			while(serial.waitForMessage(&reply, CMT_MID_GOTOCONFIGACK, 100,  1) != XRV_OK){
				serial.writeMessage(&msg);
			}

			msg.setMessageId(CMT_MID_SETPERIOD);
			msg.setDataShort(1152);
			if (serial.writeMessage(&msg)){
				std::cout << "Failed to set period" << std::endl;
				return false;
			}
			if (serial.waitForMessage(&reply, CMT_MID_SETPERIODACK, 0,  1) != XRV_OK){
				std::cout << "Failed to set period" << std::endl;
		    	return false;
			}

			msg.setMessageId(CMT_MID_RESETORIENTATION);
			msg.setDataShort(CMT_RESETORIENTATION_ALIGN);
			if (serial.writeMessage(&msg)){
				std::cout << "Failed to align" << std::endl;
		    	return false;
			}
			if (serial.waitForMessage(&reply, CMT_MID_RESETORIENTATIONACK, 0,  1) != XRV_OK){
				std::cout << "Failed to align" << std::endl;
		    	return false;
			}

			if (mode == Mode::CalibratedData){
				msg.setMessageId(CMT_MID_SETOUTPUTMODE);
				msg.setDataShort(CMT_OUTPUTMODE_CALIB);
				if (serial.writeMessage(&msg)){
					std::cout << "Failed to set output mode" << std::endl;
			    	return false;
			    }
				if (serial.waitForMessage(&reply, CMT_MID_SETOUTPUTMODEACK, 0,  1) != XRV_OK){
					std::cout << "Failed to set output mode" << std::endl;
			    	return false;
			    }

				msg.setMessageId(CMT_MID_SETOUTPUTSETTINGS);
				msg.setDataLong(CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT);
				if (serial.writeMessage(&msg)){
					std::cout << "Failed to set output format" << std::endl;
			    	return false;
				}
				if (serial.waitForMessage(&reply, CMT_MID_SETOUTPUTSETTINGSACK, 0,  1) != XRV_OK){
					std::cout << "Failed to set output format" << std::endl;
			    	return false;
				}

			} else if (mode == Mode::OrientationEstimate){
				msg.setMessageId(CMT_MID_SETOUTPUTMODE);
					msg.setDataShort(CMT_OUTPUTMODE_ORIENT);
					if (serial.writeMessage(&msg)){
						std::cout << "Failed to set output mode" << std::endl;
				    	return false;
					}
					if (serial.waitForMessage(&reply, CMT_MID_SETOUTPUTMODEACK, 0,  1) != XRV_OK){
						std::cout << "Failed to set output mode" << std::endl;
				    	return false;
					}

					msg.setMessageId(CMT_MID_SETOUTPUTSETTINGS);
					msg.setDataLong(CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT);
					if (serial.writeMessage(&msg)){
						std::cout << "Failed to set output format" << std::endl;
				    	return false;
					}
					if (serial.waitForMessage(&reply, CMT_MID_SETOUTPUTSETTINGSACK, 0,  1) != XRV_OK){
						std::cout << "Failed to set output format" << std::endl;
				    	return false;
					}
			} else if (mode == Mode::All){
				msg.setMessageId(CMT_MID_SETOUTPUTMODE);
					msg.setDataShort(CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_CALIB);
					if (serial.writeMessage(&msg)){
						std::cout << "Failed to set output mode" << std::endl;
				    	return false;
					}
					if (serial.waitForMessage(&reply, CMT_MID_SETOUTPUTMODEACK, 0,  1) != XRV_OK){
						std::cout << "Failed to set output mode" << std::endl;
				    	return false;
					}

					msg.setMessageId(CMT_MID_SETOUTPUTSETTINGS);
					msg.setDataLong(CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG | CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT);
					if (serial.writeMessage(&msg)){
						std::cout << "Failed to set output format" << std::endl;
				    	return false;
					}
					if (serial.waitForMessage(&reply, CMT_MID_SETOUTPUTSETTINGSACK, 0,  1) != XRV_OK){
						std::cout << "Failed to set output format" << std::endl;
				    	return false;
					}
			} else {
				std::cout << "Unrecognised mode" << std::endl;
				return false;
			}

			msg.setMessageId(CMT_MID_GOTOMEASUREMENT);
			msg.resizeData(0);
			if (serial.writeMessage(&msg)){
				std::cout << "Failed to set measure mode" << std::endl;
				return false;
			}
			if (serial.waitForMessage(&reply, CMT_MID_GOTOMEASUREMENTACK, 0,  1) != XRV_OK){
				std::cout << "Failed to set measure mode" << std::endl;
				return false;
			}
			return true;
		}

		~PyHALXsens(){}

		bool get_gyro_acc(py::array_t<float, py::array::c_style | py::array::forcecast> rotvel,
     					  py::array_t<float, py::array::c_style | py::array::forcecast> linacc){
     		if ((mode != Mode::CalibratedData) and (mode != Mode::All)) return false;
			auto rotvel_ptr = (float*)rotvel.request().ptr;
			auto linacc_ptr = (float*)linacc.request().ptr;
			//auto gil_release = py::gil_scoped_release();
			auto ret = serial.waitForMessage(&reply, 0, 0, 1);
			if (ret != XRV_OK) return false;
			rotvel_ptr[0] = reply.getDataFloat(3*4);      // [rad/s]
			rotvel_ptr[1] = reply.getDataFloat(4*4);      // [rad/s]
			rotvel_ptr[2] = reply.getDataFloat(5*4);      // [rad/s]
			linacc_ptr[0] = reply.getDataFloat(0*4);        // [m/s2]
			linacc_ptr[1] = reply.getDataFloat(1*4);        // [m/s2]
			linacc_ptr[2] = reply.getDataFloat(2*4);        // [m/s2]
			return true;
		}

		bool get_gyro_acc_mag(py::array_t<float, py::array::c_style | py::array::forcecast> rotvel,
     					  py::array_t<float, py::array::c_style | py::array::forcecast> linacc,
     					  py::array_t<float, py::array::c_style | py::array::forcecast> magfld){
     		if ((mode != Mode::CalibratedData) and (mode != Mode::All)) return false;
			auto rotvel_ptr = (float*)rotvel.request().ptr;
			auto linacc_ptr = (float*)linacc.request().ptr;
			auto magfld_ptr = (float*)magfld.request().ptr;
			//auto gil_release = py::gil_scoped_release();
			auto ret = serial.waitForMessage(&reply, 0, 0, 1);
			if (ret != XRV_OK) return false;
			rotvel_ptr[0] = reply.getDataFloat(3*4);      // [rad/s]
			rotvel_ptr[1] = reply.getDataFloat(4*4);      // [rad/s]
			rotvel_ptr[2] = reply.getDataFloat(5*4);      // [rad/s]
			linacc_ptr[0] = reply.getDataFloat(0*4);        // [m/s2]
			linacc_ptr[1] = reply.getDataFloat(1*4);        // [m/s2]
			linacc_ptr[2] = reply.getDataFloat(2*4);        // [m/s2]
			magfld_ptr[0] = reply.getDataFloat(6*4);        // [arbitrary, normalized to earth field strength]
			magfld_ptr[1] = reply.getDataFloat(7*4);        // [arbitrary, normalized to earth field strength]
			magfld_ptr[2] = reply.getDataFloat(8*4);        // [arbitrary, normalized to earth field strength]
			return true;
		}

		bool get_rot(py::array_t<float, py::array::c_style | py::array::forcecast> rotmat){
     		if ((mode != Mode::OrientationEstimate) and (mode != Mode::All)) return false;
			auto rotmat_ptr = (float*)rotmat.request().ptr;
			//auto gil_release = py::gil_scoped_release();
			auto ret = serial.waitForMessage(&reply, 0, 0, 1);
			if (ret != XRV_OK) return false;
			int off = 0;
			if (mode == Mode::All) off = 9;
			rotmat_ptr[0] = reply.getDataFloat((off+0)*4);
			rotmat_ptr[1] = reply.getDataFloat((off+3)*4);
			rotmat_ptr[2] = reply.getDataFloat((off+6)*4);
			rotmat_ptr[3] = reply.getDataFloat((off+1)*4);
			rotmat_ptr[4] = reply.getDataFloat((off+4)*4);
			rotmat_ptr[5] = reply.getDataFloat((off+7)*4);
			rotmat_ptr[6] = reply.getDataFloat((off+2)*4);
			rotmat_ptr[7] = reply.getDataFloat((off+5)*4);
			rotmat_ptr[8] = reply.getDataFloat((off+8)*4);
			return true;
		}

		bool get_gyro_acc_mag_rot(py::array_t<float, py::array::c_style | py::array::forcecast> rotvel,
     					  py::array_t<float, py::array::c_style | py::array::forcecast> linacc,
     					  py::array_t<float, py::array::c_style | py::array::forcecast> magfld,
						  py::array_t<float, py::array::c_style | py::array::forcecast> rotmat){
     		if (mode != Mode::All) return false;
			auto rotvel_ptr = (float*)rotvel.request().ptr;
			auto linacc_ptr = (float*)linacc.request().ptr;
			auto magfld_ptr = (float*)magfld.request().ptr;
			auto rotmat_ptr = (float*)rotmat.request().ptr;
			//auto gil_release = py::gil_scoped_release();
			auto ret = serial.waitForMessage(&reply, 0, 0, 1);
			if (ret != XRV_OK) return false;
			rotvel_ptr[0] = reply.getDataFloat(3*4);      // [rad/s]
			rotvel_ptr[1] = reply.getDataFloat(4*4);      // [rad/s]
			rotvel_ptr[2] = reply.getDataFloat(5*4);      // [rad/s]
			linacc_ptr[0] = reply.getDataFloat(0*4);        // [m/s2]
			linacc_ptr[1] = reply.getDataFloat(1*4);        // [m/s2]
			linacc_ptr[2] = reply.getDataFloat(2*4);        // [m/s2]
			magfld_ptr[0] = reply.getDataFloat(6*4);        // [arbitrary, normalized to earth field strength]
			magfld_ptr[1] = reply.getDataFloat(7*4);        // [arbitrary, normalized to earth field strength]
			magfld_ptr[2] = reply.getDataFloat(8*4);        // [arbitrary, normalized to earth field strength]
			int off = 9;
			rotmat_ptr[0] = reply.getDataFloat((off+0)*4);
			rotmat_ptr[1] = reply.getDataFloat((off+3)*4);
			rotmat_ptr[2] = reply.getDataFloat((off+6)*4);
			rotmat_ptr[3] = reply.getDataFloat((off+1)*4);
			rotmat_ptr[4] = reply.getDataFloat((off+4)*4);
			rotmat_ptr[5] = reply.getDataFloat((off+7)*4);
			rotmat_ptr[6] = reply.getDataFloat((off+2)*4);
			rotmat_ptr[7] = reply.getDataFloat((off+5)*4);
			rotmat_ptr[8] = reply.getDataFloat((off+8)*4);
			return true;
		}

		bool has_mag(){
			if ((mode != Mode::CalibratedData) or (mode != Mode::All)) return false;
			return true;
		}

		bool has_raw(){
			if ((mode != Mode::CalibratedData) or (mode != Mode::All)) return false;
			return true;
		}

		bool has_rot(){
			if ((mode != Mode::OrientationEstimate) or (mode != Mode::All)) return false;
			return true;
		}
};

PYBIND11_MODULE(xsens_MTi28A, m) {
    py::class_<PyHALXsens>(m, "HAL")
		.def(py::init<>())
		.def("init", &PyHALXsens::init)
		.def("get_gyro_acc", &PyHALXsens::get_gyro_acc)
		.def("get_gyro_acc_mag", &PyHALXsens::get_gyro_acc_mag)
		.def("get_gyro_acc_mag_rot", &PyHALXsens::get_gyro_acc_mag_rot)
		.def("get_rot", &PyHALXsens::get_rot)
		.def("has_raw", &PyHALXsens::has_raw)
		.def("has_mag", &PyHALXsens::has_mag)
		.def("has_rot", &PyHALXsens::has_rot);
	py::enum_<Mode>(m, "Mode")
		.value("CalibratedData", CalibratedData)
		.value("OrientationEstimate", OrientationEstimate)
		.value("All", All)
		.export_values();
}
