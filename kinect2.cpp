// dllmain.cpp : Defines the entry point for the DLL application.

#define EXPORT  __declspec( dllexport ) 

#include <Kinect.h>

#include <stdexcept>
#include <memory>
#include <iostream>
#include <vector>

// helpers
template<class T>
class wrapped {
protected:
	using ptr_type = T*;
	ptr_type ptr;

public:
	explicit operator bool() const { return ptr != nullptr; }
	explicit operator T* () const { return ptr; }
	ptr_type& get() { return ptr; }

	wrapped() : ptr(nullptr) {}
	wrapped(const wrapped&) = delete;
	wrapped(wrapped&& other)
		: ptr(other.ptr) {
		other.ptr = nullptr;
	}

	wrapped& operator=(const wrapped&) = delete;
	wrapped& operator=(wrapped&& other) = delete;

	~wrapped() {
		if (ptr) {
			// std::clog << "release " << ptr << std::endl;
			ptr->Release();
		}
	}

};



// kinect api
class sensor : public wrapped<IKinectSensor> {

public:

	sensor() {

		if (GetDefaultKinectSensor(&ptr) < 0) {
			throw std::runtime_error("no kinect sensor found");
		}

		if (ptr->Open() < 0) {
			throw std::runtime_error("sensor open failed");
		}

		// std::clog << "sensor ok" << std::endl;
	}

	~sensor() {
		// std::clog << "closing sensor" << std::endl;
		if (ptr) ptr->Close();

	}

};





class body_frame_reader;
class color_frame_reader;

class body : public wrapped<IBody> {
public:
	using array = body[BODY_COUNT];
};

class body_frame : public wrapped<IBodyFrame> {

public:

	using time_type = INT64;

	time_type time() const {
		time_type res;
		if (ptr->get_RelativeTime(&res) < 0) {
			throw std::runtime_error("get relative time failed");
		}
		return res;
	}


	void refresh(body::array bodies) {

		if (ptr->GetAndRefreshBodyData(BODY_COUNT, reinterpret_cast<IBody**>(bodies))) {
			throw std::runtime_error("refresh body data failed");
		}

	}

};


class waitable {

protected:
	WAITABLE_HANDLE handle;
public:
	void wait() {
		WaitForSingleObject(reinterpret_cast<HANDLE>(handle), INFINITE);
	}

	template<class ... Waitable>
	static void wait_all(Waitable&& ... args) {
		HANDLE events[] = { reinterpret_cast<HANDLE>(args.handle)... };
		WaitForMultipleObjects(sizeof...(Waitable), events, true, INFINITE);
	}

};


class color_frame : public wrapped<IColorFrame> {

};



class multi_frame : public wrapped<IMultiSourceFrame> { 

public:
	
	class color_frame color_frame() {

		wrapped<IColorFrameReference> ref;
		if (ptr->get_ColorFrameReference(&ref.get()) < 0) {
			throw std::runtime_error("color frame ref failed");
		}

		class color_frame res;
		if (ref.get()->AcquireFrame(&res.get()) < 0) {
			throw std::runtime_error("acquire color frame failed");
		}

		return res;
	}

	class body_frame body_frame() {

		wrapped<IBodyFrameReference> ref;
		if (ptr->get_BodyFrameReference(&ref.get()) < 0) {
			throw std::runtime_error("body frame ref failed");
		}

		class body_frame res;
		if (ref.get()->AcquireFrame(&res.get()) < 0) {
			throw std::runtime_error("acquire body frame failed");
		}

		return res;
	}

};


class multi_frame_reader : public wrapped<IMultiSourceFrameReader>, public waitable {
	using sensor_type = std::shared_ptr<class sensor>;
	sensor_type sensor;
public:
	const DWORD flags;
	multi_frame_reader(sensor_type sensor, DWORD flags)
		: sensor(sensor),
		flags(flags)
	{
		if (sensor->get()->OpenMultiSourceFrameReader(flags, &ptr) < 0) {
			throw std::runtime_error("open multi source frame reader failed");
		}

		ptr->SubscribeMultiSourceFrameArrived(&handle);
	}

	multi_frame latest_frame() {

		wrapped<IMultiSourceFrameArrivedEventArgs> args;

		if (ptr->GetMultiSourceFrameArrivedEventData(handle, &args.get()) < 0) {
			throw std::runtime_error("multi frame event data failed");
		}

		wrapped<IMultiSourceFrameReference> ref;
		if (args.get()->get_FrameReference(&ref.get()) < 0) {
			throw std::runtime_error("multi frame ref failed");
		}

		multi_frame res;
		if (ref.get()->AcquireFrame(&res.get()) < 0) {
			throw std::runtime_error("acquire multi frame failed");
		}

		return res;
	}

	~multi_frame_reader() {
		ptr->UnsubscribeMultiSourceFrameArrived(handle);
	}
};




// TODO coordinate mapper

namespace capi {

	struct vec3 { float x, y, z; };
	// struct bgra { char b, g, r, a;  };

	struct body {
		unsigned index;

		// TODO more
		vec3 joint[JointType_Count];

		using callback = void(*)(body* data, unsigned size);
	};

	struct color {
		int width, height;
		RGBQUAD* data;

		using callback = void(*)(color img);
	};

	struct instance {

		std::shared_ptr<multi_frame_reader> reader;

		body::callback body_cb = nullptr;
		color::callback color_cb = nullptr;

		std::vector<RGBQUAD> buffer;
	};

}



extern "C" {

	EXPORT capi::instance* init(DWORD flags);
	EXPORT void release(capi::instance*);

	EXPORT void body_callback(capi::instance* instance, capi::body::callback cb) {
		instance->body_cb = cb;
	}

	EXPORT void color_callback(capi::instance* instance, capi::color::callback cb) {
		instance->color_cb = cb;
	}

	EXPORT void update(capi::instance*);

}

capi::instance* init(DWORD flags) {
	try{
		capi::instance* res = new capi::instance;

		auto sensor = std::make_shared<class sensor>();

		res->reader = std::make_shared < multi_frame_reader >(sensor, flags);

		return res;
	}
	catch (std::runtime_error&) {
		return nullptr;
	}

}

void release(capi::instance* instance) {
	// std::cout << "release" << std::endl;
	delete instance;
}


static void on_body_frame(body_frame&& frame, capi::body::callback cb) {
	// std::cout << "on_latest_frame " << instance << std::endl;

	body::array bodies;
	frame.refresh(bodies);

	capi::body cbodies[BODY_COUNT];
	unsigned off = 0;

	for (unsigned i = 0; i < BODY_COUNT; ++i) {

		if (bodies[i]) {

			BOOLEAN tracked;
			bodies[i].get()->get_IsTracked(&tracked);

			// std::clog << "body " << i << " tracked: " << tracked << std::endl;

			if (tracked) {
				Joint joints[JointType_Count];

				if (bodies[i].get()->GetJoints(JointType_Count, joints) < 0) {
					throw std::runtime_error("cannot get joints");
				}

				cbodies[off].index = i;

				for (unsigned j = 0; j < JointType_Count; ++j) {
					auto pos = joints[j].Position;
					cbodies[off].joint[j] = { pos.X, pos.Y, pos.Z };
				}

				++off;
			}
		}
	}

	// callback
	cb(cbodies, off);
}



static void on_color_frame(capi::instance* instance, color_frame&& frame, capi::color::callback cb) {
	
	wrapped<IFrameDescription> desc;

	if (frame.get()->get_FrameDescription(&desc.get()) < 0) {
		throw std::runtime_error("frame description failed");
	}

	capi::color img;

	// TODO error check
	desc.get()->get_Width(&img.width);
	desc.get()->get_Height(&img.height);

	ColorImageFormat format = ColorImageFormat_None;
	frame.get()->get_RawColorImageFormat(&format);

	// std::cout << "c++: " << img.width << " " << img.height << " " << img.data << std::endl;

	UINT size = 0;

	if (format == ColorImageFormat_Bgra) {
		frame.get()->AccessRawUnderlyingBuffer(&size, reinterpret_cast<BYTE**>(&img.data));
	}
	else {
		instance->buffer.resize(img.width * img.height);
		size = instance->buffer.size() * sizeof(RGBQUAD);

		if (frame.get()->CopyConvertedFrameDataToArray(size,
			reinterpret_cast<BYTE*>(instance->buffer.data()),
			ColorImageFormat_Bgra) < 0) {
			throw std::runtime_error("frame copy failed");
		}

		img.data = instance->buffer.data();
	}

	// callback
	cb(img);

}


void update(capi::instance* instance) {

	try {
		instance->reader->wait();
		auto frame = instance->reader->latest_frame();

		if ((instance->reader->flags & FrameSourceTypes::FrameSourceTypes_Color) && instance->color_cb) {
			on_color_frame(instance, frame.color_frame(), instance->color_cb);
		}

		if ((instance->reader->flags & FrameSourceTypes::FrameSourceTypes_Body) && instance->body_cb) {
			on_body_frame(frame.body_frame(), instance->body_cb);
		}
		
	}
	catch (std::runtime_error& e) {
		std::cerr << "multi frame error: " << e.what() << std::endl;
	}

}