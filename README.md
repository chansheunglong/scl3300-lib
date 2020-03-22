# muRata SCL3300 Library for Arduino

This is a library for the muRata SCL3300 L High Performance 3-axis Inclinometer
--> https://www.murata.com/en-global/products/sensor/inclinometer/scl3300

Current Version: 1.0.0

Author: Chan Sheung Long
------------

#### Changelog
2020/02/07  : Initial Release (Version 1.0.0)

------------
#### Main Functions
    void setCSPin(int pinNum);
    void startup();
    void sleep();
    void wake();
    void mode(int mode);
    bool angle(float &ANG_X, float &ANG_Y, float &ANG_Z);
    bool arc_angle(int32_t &ARC_X, int32_t &ARC_Y, int32_t &ARC_Z);
    float temperature();
    void setZero(float zeroX = 0, float zeroY = 0, float zeroZ = 0);
    void reset();
    float arcsecond_to_degree(int32_t arc_angle);

------------

This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
