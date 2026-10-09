#
#  [2024] SubSeaPulse SRL
#  All Rights Reserved.
#
#  Authors Filippo Campagnaro
#  
# This is free software: you can redistribute it and/or modify it        
# under the terms of the GNU General Public License version 3 as         
# published by the Free Software Foundation.                             
#                                                                        
# This program is distributed in the hope that it will be useful, but    
# WITHOUT ANY WARRANTY; without even the implied warranty of FITNESS     
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for       
# more details.                                                          
#                                                                        
# You should have received a copy of the GNU General Public License      
# along with this program. If not, see <http://www.gnu.org/licenses/>.   
#


AM_CXXFLAGS = @SUMDEMO_CXXFLAGS@

lib_LTLIBRARIES = libjanus_wrap.la

libjanus_wrap_la_SOURCES = janus-wrapper.cpp janus-mac.cpp
libjanus_wrap_la_CPPFLAGS = @SUMDEMO_CPPFLAGS@  
libjanus_wrap_la_LDFLAGS = @SUMDEMO_LDFLAGS@ 
libjanus_wrap_la_LIBADD = @SUMDEMO_LIBADD@ @JANUS_LDADD@ @ALSA_LDADD@
