#ifndef MABUR_PLAYER_OSD_WANTED_H_
#define MABUR_PLAYER_OSD_WANTED_H_

namespace maburplay {

// Whether the presenter claims the OSD plane at all. Any one of the three
// layers is reason enough: with both overlays off and the buttons wired, the
// settings menu still needs somewhere to draw.
inline bool osd_plane_wanted(bool msp, bool gs, bool menu) { return msp || gs || menu; }

}  // namespace maburplay

#endif  // MABUR_PLAYER_OSD_WANTED_H_
