# Custom OSD fonts

The web GS offers extra OSD fonts in the Config form's Display group. They are listed in
`web/ui/public/fonts/index.json`. The default Betaflight atlas (`font_btfl.png`) is always available.

| Picker name | File | Description |
|---|---|---|
| PADDY octa S | `fonts/paddy_octa_s.png` | Octagonal letters with an inline groove, sized to match the stock letters |
| PADDY octa L | `fonts/paddy_octa_l.png` | The same design, larger and bolder |
| PADDY green | `fonts/paddy_green.png` | Thin neon-green fighter-HUD style, all-green icons |

## Source and licence

Letters, digits, punctuation and most icons in these fonts are original artwork by PaddyP90,
generated procedurally and released under the MIT licence. Glyphs that were not redrawn (logo tiles and a few
icons such as the speedometer, drone and mountain symbols) come unchanged from the stock Betaflight
atlas and keep its licence. The atlases are produced from `.mfont` files with `tools/msp/gen_webfont.py`.
