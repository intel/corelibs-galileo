/*
Server.h header copied from Ethernet library and changed by Intel

Copyright (c) Arduino LLC. All right reserved.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

*/

#ifndef server_h
#define server_h

class Server : public Print {
public:
  virtual void begin() =0;
};

#endif
