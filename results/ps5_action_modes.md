# PS5: modes of the Push-T diffusion policy

64 samples per query from the authors' pretrained checkpoint, seed 0. Pusher positions are in the T frame (pixels); velocities in pixels per second. Separation is the distance between the two k-means centers of the sequence endpoints divided by the RMS spread within the clusters; a query counts as bimodal when it is above 4.0 with at least 10% of the samples in the smaller cluster. Headings are the directions of the two cluster centers from the pusher, in image coordinates (y down).

| T pose | pusher (T frame) | modes | separation | minor share | mean reach (px) | cluster headings (deg) |
|---|---|---|---|---|---|---|
| T at its initial pose | (0, -100) | 2 | 10.6 | 44% | 175 | 100, 160 |
| T at its initial pose | (100, -100) | 1 | 2.4 | 30% | 215 | 145, 142 |
| T at its initial pose | (100, 50) | 1 | 2.4 | 12% | 56 | 164, -164 |
| T at its initial pose | (50, 50) | 1 | 1.8 | 33% | 112 | 166, 160 |
| T at its initial pose | (0, -300) | 1 | 2.9 | 16% | 329 | 142, 150 |
| T at its initial pose | (0, -100), v = (100, 0) | 1 | 1.9 | 44% | 195 | 108, 110 |
| T at its initial pose | (0, -100), v = (-100, 0) | 1 | 1.9 | 48% | 194 | 157, 151 |
| T displaced 100 px in +x | (0, -100) | 1 | 2.6 | 42% | 129 | 105, 96 |
| T displaced 100 px in +x | (100, -100) | 1 | 2.2 | 45% | 146 | 155, 150 |
| T displaced 100 px in +x | (100, 50) | 1 | 2.0 | 34% | 84 | -130, -131 |
| T displaced 100 px in +x | (50, 50) | 1 | 1.8 | 38% | 79 | -136, -138 |
| T displaced 100 px in +x | (0, -300) | 1 | 2.0 | 36% | 291 | 130, 126 |
| T displaced 100 px in +x | (0, -100), v = (100, 0) | 1 | 2.6 | 31% | 150 | 104, 100 |
| T displaced 100 px in +x | (0, -100), v = (-100, 0) | 1 | 2.7 | 47% | 127 | 103, 92 |
