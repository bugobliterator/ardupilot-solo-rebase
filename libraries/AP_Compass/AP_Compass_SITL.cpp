#include "AP_Compass_SITL.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <uavcan/equipment/ahrs/MagneticFieldStrength2.hpp>

extern const AP_HAL::HAL& hal;

AP_Compass_SITL::AP_Compass_SITL()
    : _sitl(AP::sitl())
{
    if (_sitl != nullptr) {
        _compass._setup_earth_field();
        for (uint8_t i=0; i<MAX_CONNECTED_MAGS; i++) {
            uint32_t dev_id = _sitl->mag_devid[i];
            if (dev_id == 0) {
                continue;
            }
            uint8_t instance;
            AP_HAL::Device::DeviceStructure dev_id_s = AP_HAL::Device::get_dev_id_struct(dev_id);
            if (dev_id_s.bus_type == AP_HAL::Device::BUS_TYPE_UAVCAN) {
                _compass_ucnode[i] = static_cast<const HAL_SITL*>(&hal)->_sitl_state->initNode(dev_id_s.bus, dev_id_s.address);
                _compass_ucnode_sensor_id[i] = dev_id_s.devtype - 1;
                continue;
            }
            if (!register_compass(dev_id, instance)) {
                continue;
            } else if (_num_nonuc_compass<MAX_SITL_COMPASSES) {
                _compass_nonuc_instance[_num_nonuc_compass] = instance;
                set_dev_id(_compass_nonuc_instance[_num_nonuc_compass], dev_id);

                // save so the compass always comes up configured in SITL
                save_dev_id(_compass_nonuc_instance[_num_nonuc_compass]);
                _num_nonuc_compass++;
            }
        }

        // Scroll through the registered compasses, and set the offsets
        for (uint8_t i=0; i<_num_nonuc_compass; i++) {
            if (_compass.get_offsets(i).is_zero()) {
                _compass.set_offsets(i, _sitl->mag_ofs);
            }
        }

        // make first compass external
        set_external(_compass_nonuc_instance[0], true);

        hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &AP_Compass_SITL::_timer, void));
    }
}


/*
  create correction matrix for diagnonals and off-diagonals
*/
void AP_Compass_SITL::_setup_eliptical_correcion(void)
{
    Vector3f diag = _sitl->mag_diag.get();
    if (diag.is_zero()) {
        diag(1,1,1);
    }
    const Vector3f &diagonals = diag;
    const Vector3f &offdiagonals = _sitl->mag_offdiag;
    
    if (diagonals == _last_dia && offdiagonals == _last_odi) {
        return;
    }
    
    _eliptical_corr = Matrix3f(diagonals.x,    offdiagonals.x, offdiagonals.y,
                               offdiagonals.x, diagonals.y,    offdiagonals.z,
                               offdiagonals.y, offdiagonals.z, diagonals.z);
    if (!_eliptical_corr.invert()) {
        _eliptical_corr.identity();
    }
    _last_dia = diag;
    _last_odi = offdiagonals;
}

void AP_Compass_SITL::_timer()
{
    // TODO: Refactor delay buffer with AP_Baro_SITL.

    // Sampled at 100Hz
    uint32_t now = AP_HAL::millis();
    if ((now - _last_sample_time) < 10) {
        return;
    }
    _last_sample_time = now;

    // calculate sensor noise and add to 'truth' field in body frame
    // units are milli-Gauss
    Vector3f noise = rand_vec3f() * _sitl->mag_noise;
    Vector3f new_mag_data = _sitl->state.bodyMagField + noise;

    // add delay
    uint32_t best_time_delta = 1000; // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index = 0; // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now - last_store_time >= 10) { // store data every 10 ms.
        last_store_time = now;
        if (store_index > buffer_length-1) { // reset buffer index if index greater than size of buffer
            store_index = 0;
        }
        buffer[store_index].data = new_mag_data; // add data to current index
        buffer[store_index].time = last_store_time; // add time to current index
        store_index = store_index + 1; // increment index
    }

    // return delayed measurement
    uint32_t delayed_time = now - _sitl->mag_delay; // get time corresponding to delay
    // find data corresponding to delayed time in buffer
    for (uint8_t i=0; i<=buffer_length-1; i++) {
        // find difference between delayed time and time stamp in buffer
        uint32_t time_delta = abs((int32_t)(delayed_time - buffer[i].time));
        // if this difference is smaller than last delta, store this time
        if (time_delta < best_time_delta) {
            best_index= i;
            best_time_delta = time_delta;
        }
    }
    if (best_time_delta < 1000) { // only output stored state if < 1 sec retrieval error
        new_mag_data = buffer[best_index].data;
    }

    _setup_eliptical_correcion();        
    
    new_mag_data = _eliptical_corr * new_mag_data;
    new_mag_data -= _sitl->mag_ofs.get();

    for (uint8_t i=0; i<_num_nonuc_compass; i++) {
        Vector3f f = new_mag_data;
        if (i == 0) {
            // rotate the first compass, allowing for testing of external compass rotation
            f.rotate_inverse((enum Rotation)_sitl->mag_orient.get());
            f.rotate(get_board_orientation());

            // scale the first compass to simulate sensor scale factor errors
            f *= _sitl->mag_scaling;
        }
        accumulate_sample(f, _compass_nonuc_instance[i], 10);
    }

    for (uint8_t i = 0; i < MAX_CONNECTED_MAGS; i++) {
        if (_compass_ucnode[i] != nullptr) {
            uavcan::Publisher<uavcan::equipment::ahrs::MagneticFieldStrength2> mfs2_pub(*(_compass_ucnode[i]->get_node()));
            uavcan::equipment::ahrs::MagneticFieldStrength2 mfs2;
            mfs2.sensor_id = _compass_ucnode_sensor_id[i];
            mfs2.magnetic_field_ga[0] = (double)new_mag_data.x/1000.0;
            mfs2.magnetic_field_ga[1] = (double)new_mag_data.y/1000.0;
            mfs2.magnetic_field_ga[2] = (double)new_mag_data.z/1000.0;
            {
                WITH_SEMAPHORE(_compass_ucnode[i]->get_sem());
                (void)mfs2_pub.broadcast(mfs2);
            }
        }
    }

}

void AP_Compass_SITL::read()
{
    for (uint8_t i=0; i<_num_nonuc_compass; i++) {
        drain_accumulated_samples(_compass_nonuc_instance[i], nullptr);
    }
}
#endif
