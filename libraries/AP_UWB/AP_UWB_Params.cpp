/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_UWB_Params.h"
#include "AP_UWB.h"


// table of user settable parameters
const AP_Param::GroupInfo AP_UWB_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: UWB type
    // @Description: Type of connected UWB
    // @Values: 0:None,1:FLNC
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_UWB_Params, type, (float)Type::NONE, AP_PARAM_FLAG_ENABLE),


    AP_GROUPEND
};

AP_UWB_Params::AP_UWB_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
