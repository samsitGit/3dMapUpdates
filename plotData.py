import matplotlib.pyplot as plt

# Data for each category, replace these with your actual data
frames = list(range(699))  # Assuming continuous frames from 0 to 698

# Your provided data for each category
mapString = "174 166 173 171 163 152 147 147 194 195 195 184 186 193 189 188 191 192 200 195 194 204 219 201 213 215 219 234 252 293 256 265 232 219 190 218 203 198 216 199 199 222 273 218 191 165 205 180 182 180 188 194 204 213 216 206 204 204 201 189 198 192 208 209 243 225 214 220 439 358 286 256 283 309 277 268 263 287 221 220 210 223 234 258 263 251 231 267 216 204 191 187 188 189 190 192 193 194 189 195 209 200 187 191 188 189 183 191 157 171 169 177 176 213 270 316 230 197 192 200 186 186 185 187 191 188 180 194 188 202 210 292 268 232 215 189 204 201 202 213 238 230 230 219 218 195 200 191 213 208 216 207 217 220 206 204 185 208 198 193 190 189 194 187 194 183 196 215 206 194 192 191 188 187 188 189 200 198 191 191 186 239 256 286 340 308 285 260 251 253 250 247 266 272 285 305 427 263 191 185 191 203 200 199 190 184 191 190 189 190 206 199 213 195 196 191 195 196 190 204 195 195 182 189 192 188 189 189 191 187 197 187 184 195 184 187 187 194 196 190 186 187 188 177 174 185 179 186 184 182 184 193 192 189 188 182 187 193 186 186 182 192 182 177 194 195 191 195 215 226 213 226 212 216 197 210 207 200 198 196 200 197 195 213 199 194 196 192 193 191 197 207 230 206 201 198 199 191 192 193 192 189 196 189 204 232 255 424 206 198 186 186 197 225 248 206 195 191 185 173 183 195 187 185 182 180 194 186 159 164 166 189 224 261 142 230 170 159 153 198 198 177 194 181 173 166 179 175 176 173 177 171 173 183 170 169 160 171 165 174 167 169 165 164 185 167 188 170 164 167 169 178 161 161 174 169 177 184 178 176 182 162 185 168 164 165 179 168 176 179 174 181 177 184 168 191 170 181 171 177 172 168 173 175 175 172 175 181 174 178 168 174 177 176 178 180 187 195 187 156 196 179 181 181 177 190 237 268 310 376 264 283 253 209 192 190 189 203 182 194 190 205 162 157 156 142 154 145 166 170 173 208 179 193 202 196 192 191 232 186 211 217 214 211 204 192 212 208 239 221 224 217 257 219 224 213 215 214 210 219 204 211 201 205 225 217 211 201 205 201 207 210 197 199 209 206 201 192 207 225 218 204 192 205 189 205 216 221 224 225 217 216 208 195 204 216 262 244 221 221 218 218 249 237 260 226 210 314 191 203 211 256 357 148 149 205 180 170 173 160 171 167 176 180 181 184 180 189 180 181 184 184 192 194 204 197 199 180 194 195 198 206 190 187 203 198 202 189 192 208 196 184 196 186 197 175 190 212 177 191 190 183 198 191 195 187 218 208 195 186 178 183 179 176 183 189 196 188 190 190 190 184 183 179 182 175 171 187 190 177 192 193 180 208 226 446 189 230 192 174 186 200 167 183 199 191 194 196 178 199 209 197 196 185 188 189 194 192 189 222 216 199 197 201 221 401 202 200 180 192 203 198 211 179 190 197 184 199 192 199 199 189 199 200 186 197 226 219 182 192 193 174 163 170 159 166 170 167 178 176 176 184 185 181 190 201 199 198 210 208 181 188 190 186 188 186 203 245 272"
appearEx = "23 20 22 20 23 19 20 18 20 21 20 19 20 23 20 18 18 20 19 22 20 21 21 20 22 21 22 23 28 28 24 24 24 21 21 19 22 19 20 19 18 20 36 25 23 19 29 18 19 17 19 19 21 21 20 23 18 22 19 18 17 19 20 22 24 21 21 21 25 29 38 24 25 29 42 51 37 57 36 32 37 37 36 58 67 55 53 43 32 33 30 25 23 25 24 26 24 24 33 33 28 27 20 24 27 26 23 27 25 32 30 23 26 23 34 49 41 30 27 22 25 22 26 27 26 28 24 25 30 34 37 35 34 34 38 30 28 35 30 36 30 28 30 29 32 30 25 29 26 26 27 34 25 30 33 28 21 23 26 23 19 26 31 27 26 29 29 41 29 27 25 21 31 27 21 21 38 32 26 27 25 28 30 30 35 70 58 60 53 60 48 53 65 53 41 44 71 45 34 48 34 29 34 59 68 37 35 76 47 41 44 45 49 49 57 37 50 37 43 29 35 30 29 22 25 27 26 25 21 24 22 25 23 24 24 20 21 36 26 20 24 21 20 22 27 29 26 23 25 30 33 33 30 33 32 31 31 31 25 21 31 27 26 35 27 21 19 17 24 20 34 26 19 19 17 18 18 28 16 20 19 20 23 19 19 18 18 19 17 22 19 20 23 22 18 21 21 28 16 21 28 23 19 23 22 22 23 22 20 18 20 18 21 24 23 32 20 18 22 20 22 18 21 19 18 17 16 20 17 21 17 17 17 23 4123 600 313 48 36 38 40 26 55 32 58 50 42 37 64 48 138 41 110 26 34 121 109 120 156 53 28 106 67 144 73 23 174 173 31 157 173 209 196 32 62 208 95 30 86 53 49 435 26 213 305 421 118 64 153 48 136 26 26 24 80 18 19 15 32 17 17 17 15 18 18 16 17 16 17 17 18 17 14 17 18 18 18 16 22 20 27 18 19 19 19 19 17 21 26 33 62 37 42 45 32 35 37 44 46 53 39 43 57 54 54 51 69 46 36 41 32 35 49 32 37 33 28 38 44 32 36 39 40 52 45 32 27 25 28 25 26 26 24 25 23 22 24 23 22 24 24 22 46 22 29 27 31 25 29 23 26 23 23 26 26 29 27 27 26 22 24 34 26 27 46 34 33 30 29 33 31 27 25 25 28 33 42 29 27 23 32 28 33 28 28 30 30 30 27 25 30 35 20 337 832 43 27 26 22 21 23 32 64 20 22 74 54 60 31 21 31 22 25 57 30 27 39 58 70 41 38 56 79 21 23 26 27 26 28 29 23 24 30 26 26 29 33 39 47 44 56 61 64 89 84 99 98 87 70 53 46 40 41 35 28 27 25 27 25 25 25 22 27 111 19 20 17 18 17 19 16 20 29 18 17 26 21 23 15 23 22 20 17 22 16 18 19 20 16 18 26 21 20 29 19 19 18 17 21 25 28 31 26 21 30 24 27 27 24 21 34 28 24 22 19 21 25 23 26 25 21 19 19 18 20 18 21 28 21 18 18 17 18 26 19 18 18 17 20 19 17 19 18 18 19 18 18 19 17 19 15 17 20 20 18 16 21 19 23"
appearUp = "0 0 0 0 0 0 0 4 0 0 0 0 0 0 4 0 0 0 0 0 0 9 0 0 0 0 0 0 14 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 2 0 0 0 0 0 0 17 0 0 0 0 0 0 42 0 0 0 0 0 0 48 0 0 0 0 0 0 43 0 0 0 0 0 0 51 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 3 0 0 0 0 0 0 10 0 0 0 0 0 0 46 0 0 0 0 0 0 51 0 0 0 0 0 0 44 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 8 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 515 0 0 0 0 0 0 1077 0 0 432 0 1050 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 12 0 0 0 0 0 0 27 0 0 0 0 0 0 45 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 13 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 11 0 0 0 0 0 0 0 0 0 0 0 0 0 3 0 0 0 0 0 0 13 0 0 0 0 0 0 36 0 0 0 0 0 0 99 0 0 0 0 0 0 107 0 0 0 0 0 0 113 0 0 0 0 0 0 0 0 0 0 0 3 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"
disEx = "130 144 144 153 141 126 138 134 123 121 125 123 126 129 116 128 131 142 138 138 137 141 148 143 146 156 154 175 222 217 221 190 189 165 153 169 151 176 156 159 160 184 211 196 153 149 164 161 146 152 148 164 164 166 151 147 164 155 148 142 143 144 162 156 167 160 144 172 181 171 164 164 200 180 178 182 198 181 160 155 159 139 162 176 174 171 167 154 144 141 129 118 128 126 137 137 140 138 144 143 149 151 145 134 132 145 157 175 144 162 168 141 147 201 277 238 181 165 160 165 157 171 163 160 162 179 204 169 195 190 208 249 267 205 204 195 192 202 220 229 238 232 218 203 203 202 195 202 190 188 181 170 175 182 194 169 168 156 164 164 151 156 151 165 159 167 163 167 174 166 167 161 164 159 166 159 165 177 158 166 167 177 185 237 230 231 235 234 227 238 246 211 222 215 216 319 242 233 201 208 209 224 221 210 201 211 209 193 200 209 205 215 210 200 192 204 193 201 196 193 194 182 182 175 181 178 172 175 171 180 166 159 160 159 169 172 173 167 160 167 168 165 157 159 152 160 161 165 156 156 149 138 147 142 150 148 155 158 155 157 148 159 157 161 166 170 193 171 196 135 139 133 133 132 129 131 128 129 136 127 145 138 139 156 136 139 131 129 142 121 120 147 127 119 116 115 123 118 118 128 119 116 140 128 127 150 148 121 118 119 111 114 130 124 171 126 124 119 111 111 119 118 117 115 113 107 113 120 108 115 113 110 109 118 85 92 89 79 465 99 207 96 94 84 110 193 91 97 93 230 96 134 288 83 352 84 87 76 87 68 3735 311 81 71 78 332 67 312 484 429 75 71 78 69 65 67 72 67 197 79 80 60 348 84 76 82 77 80 68 76 90 81 240 72 186 83 87 83 88 84 90 94 85 95 88 83 105 86 102 85 103 96 99 100 118 146 108 99 118 247 143 140 128 131 132 142 367 173 177 205 232 215 216 173 182 172 182 173 171 167 163 178 179 186 172 167 169 169 169 167 179 189 187 202 199 186 180 186 200 197 235 219 232 224 221 206 233 189 206 200 211 193 199 186 182 174 170 165 157 161 160 163 157 172 178 165 169 166 157 157 170 159 155 154 153 158 167 161 155 175 173 173 170 178 174 186 199 224 207 193 184 172 176 177 176 179 210 194 184 184 180 201 200 243 216 191 170 151 157 168 157 164 175 288 433 158 155 151 150 153 145 161 161 146 148 147 149 156 146 155 149 160 176 164 163 157 165 163 162 152 136 131 125 128 128 136 137 140 133 133 129 137 132 125 137 128 135 140 125 134 148 125 131 133 143 135 151 140 121 128 117 116 115 131 132 133 124 131 123 126 137 150 128 122 131 129 128 129 123 127 136 116 120 125 152 130 136 129 118 110 117 123 116 119 128 132 122 127 133 146 154 146 142 140 140 139 134 144 167 174 174 150 149 164 160 152 142 144 130 142 171 129 149 136 125 135 133 143 145 131 131 134 136 135 132 128 140 123 110 112 111 114 110 122 114 115 117 115 122 118 108 117 113 125 123 134 136 133 141 117 121 113 123 102 109 103 95 99 111"
disUp = "0 0 0 0 0 0 0 0 6 0 0 0 0 0 0 0 0 0 0 8 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 8 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 6 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 20 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 4 0 0 0 0 0 0 0 0 8 0 0 0 0 0 7 0 2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 8 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 133 0 0 0 0 0 0 0 0 175 0 0 5 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 9 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 8 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 4 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"
indexString = "106 116 119 129 118 100 106 102 97 98 103 101 102 104 95 103 109 112 100 107 115 115 120 111 115 130 128 146 180 182 187 160 161 137 127 140 128 147 133 135 125 155 184 171 130 127 135 124 121 115 120 140 140 141 128 125 139 132 125 117 121 123 135 131 143 136 121 142 153 144 139 139 172 147 145 141 156 153 131 129 136 115 134 145 141 138 139 127 114 103 98 94 97 99 106 107 113 111 105 115 124 121 119 111 109 114 126 134 119 137 139 117 118 175 236 205 151 137 131 139 129 146 135 129 127 146 151 145 168 163 178 216 230 178 171 169 167 173 191 197 209 198 191 176 171 175 169 176 161 162 154 144 149 158 165 134 144 133 139 139 123 126 125 141 136 142 140 143 145 138 139 136 132 136 141 134 139 144 134 136 137 152 160 200 197 197 197 197 195 198 205 179 192 182 186 281 206 202 171 179 177 191 187 182 170 179 164 165 169 178 176 188 178 172 164 175 163 174 164 164 169 152 154 150 156 147 147 141 144 152 139 134 132 133 144 147 147 139 133 141 142 138 132 129 128 132 130 140 132 133 128 114 123 110 125 125 132 133 130 130 127 137 135 138 141 147 162 144 168 111 116 113 109 107 105 110 105 101 108 105 122 115 111 130 117 107 107 108 121 101 100 121 105 98 94 95 96 95 98 103 97 94 117 107 104 122 123 100 96 97 89 90 105 99 141 103 93 95 89 93 99 98 96 95 93 86 91 95 89 92 91 85 85 95 67 74 66 60 315 75 180 72 71 64 71 173 70 75 73 209 76 66 266 59 321 62 64 57 63 48 55 53 55 48 54 306 48 46 466 47 50 51 50 45 47 47 49 47 174 48 50 42 57 54 55 54 53 57 48 55 60 53 58 54 161 61 61 54 56 61 62 67 59 64 67 63 82 62 76 65 68 74 76 76 96 77 85 78 86 77 119 114 101 109 107 117 317 134 139 173 198 183 182 143 152 141 153 143 143 141 135 146 150 156 145 132 141 138 141 135 148 156 158 168 165 158 150 155 169 169 202 190 203 172 176 170 190 155 177 163 178 158 166 148 147 141 140 136 131 135 131 136 129 139 145 140 143 137 130 126 140 133 124 127 130 132 142 130 129 146 145 144 139 150 149 161 168 187 174 162 153 142 148 142 146 151 174 169 149 152 150 164 166 206 184 159 135 125 129 129 130 133 147 262 127 132 128 127 121 119 119 117 130 122 118 119 119 131 115 124 122 126 127 136 134 132 140 125 135 117 112 107 100 104 105 113 115 114 110 110 107 114 108 104 110 105 105 114 102 110 104 99 105 109 119 106 123 115 98 100 95 92 89 104 98 90 98 93 91 102 106 112 104 92 111 103 93 106 97 93 103 96 99 101 120 108 104 100 91 89 88 94 93 84 92 108 98 104 108 115 129 117 117 114 117 115 110 116 140 147 145 125 123 133 135 120 108 117 104 104 144 106 109 111 100 111 95 102 112 102 102 101 102 108 100 93 109 91 82 84 86 88 84 95 85 89 91 91 97 88 81 88 85 87 92 98 95 95 110 92 88 88 93 77 81 79 72 74 83"
sizeString = "0 0 0 0 0 0 0 501 0 0 0 0 0 0 927 0 0 0 0 0 0 1247 0 0 0 0 0 0 1561 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 392 0 0 0 0 0 0 1629 0 0 0 0 0 0 4831 0 0 0 0 0 0 7526 0 0 0 0 0 0 8366 0 0 0 0 0 0 8206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 441 0 0 0 0 0 0 1758 0 0 0 0 0 0 5615 0 0 0 0 0 0 8502 0 0 0 0 0 0 8678 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 4065 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1640 0 0 0 0 0 0 5563 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 626 0 0 0 0 0 0 2907 0 0 0 0 0 0 7004 0 0 0 0 0 0 8634 0 0 0 0 0 0 0 0 322 2543 0 0 2719 0 0 416 4075 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 406 0 0 0 0 0 0 1193 0 0 0 0 0 0 4370 0 0 0 0 0 0 11647 0 0 0 0 0 0 17601 0 0 0 0 0 0 18553 0 0 0 0 0 0 18430 0 0 0 0 0 0 0 466 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"
map_clipping = list(map(int, mapString.split()))
appeared_cluster_extraction = list(map(int, appearEx.split()))
appeared_cluster_updates = list(map(int, appearUp.split()))
disappeared_cluster_extraction = list(map(int, disEx.split()))
indexing_vectors = list(map(int, indexString.split()))
disappeared_cluster_updates = list(map(int, disUp.split()))
frame_sizes = list(map(int, sizeString.split()))

# Sum indexing_vectors and disappeared_cluster_extraction
total_disappeared_extraction = [dce + iv for dce, iv in zip(disappeared_cluster_extraction, indexing_vectors)]

# Stacked data
stacked_data = [
    map_clipping,
    appeared_cluster_extraction,
    appeared_cluster_updates,
    total_disappeared_extraction,
    disappeared_cluster_updates
]

# Colors and labels for the stacked bar chart
colors = ['red', 'blue', 'green', 'purple', 'orange']
labels = ['Map Clipping', 'Appeared Cluster Extraction', 'Appeared Cluster Updates',
          'Disappeared Cluster Extraction + Indexing Vectors', 'Disappeared Cluster Updates']

# Plotting
fig, ax1 = plt.subplots()

# Calculate total height of bars to adjust y-limits
cumulative_height = [sum(values) for values in zip(*stacked_data)]

# Plot bars for latency data
bottom = [0] * len(frames)
for data, color, label in zip(stacked_data, colors, labels):
    ax1.bar(frames, data, bottom=bottom, label=label, color=color, width=1)
    bottom = [sum(x) for x in zip(bottom, data)]

ax1.set_xlabel('Frame')
ax1.set_ylabel('Latency Time (seconds)')
ax1.set_title('Latency and Frame Sizes per Frame')
ax1.legend(loc='upper left')

# Set limits for latency axis based on the maximum stack height
max_latency = max(cumulative_height)
ax1.set_ylim(0, max_latency * 1.1)  # ensure the top of the bar is visible

# Create a second y-axis for frame sizes
ax2 = ax1.twinx()
ax2.plot(frames, frame_sizes, label='Update Size', color='black', linewidth=2)
ax2.set_ylabel('Update Size (# of points)', color='black')
ax2.tick_params(axis='y', labelcolor='black')
ax2.legend(loc='upper right')

# Set limits for frame size axis to align zero with the first axis
max_frame_size = max(frame_sizes)
ax2.set_ylim(0, max_frame_size * (max_latency * 1.1) / max_latency)  # adjust the top limit to align the zero

plt.show()