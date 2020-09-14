# OCDots

> OCDots is a javascript library for creating evenly distributed points inside a polygon

[![codecov](https://codecov.io/gh/luxedo/ocdots/branch/master/graph/badge.svg)](https://codecov.io/gh/luxedo/ocdots) [![CodeFactor](https://www.codefactor.io/repository/github/luxedo/ocdots/badge)](https://www.codefactor.io/repository/github/luxedo/ocdots) [![npm version](https://badge.fury.io/js/ocdots.svg)](https://badge.fury.io/js/ocdots) [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

![ocdots][docs/ocdots.gif]

## Quick Start

Import the library, then call `movePoints` on a set of points to run
one iteration

```javascript
const polygon = [
  [0, 0],
  [0, 500],
  [500, 500],
  [500, 0],
  [0, 0],
];
const points = [
  [10, 10],
  [20, 20],
  [30, 30],
  [40, 40],
  [50, 50],
  [60, 60],
  [70, 70],
];
let momentum = points.map(() => [0, 0]);
const newPoints = movePoints({
  points,
  momentum,
  polygon,
}); // Points closer to the relaxed position
```

Or call `relaxPoints` to run several iterations:

```javascript
const polygon = [
  [0, 0],
  [0, 500],
  [500, 500],
  [500, 0],
  [0, 0],
];
const points = [
  [10, 10],
  [20, 20],
  [30, 30],
  [40, 40],
  [50, 50],
  [60, 60],
  [70, 70],
];
const iterations = 600;
const newPoints = relaxPoints({
  points,
  polygon,
  iterations,
}); // Points closer to the relaxed position, 600 iterations
```

## Install

Install with npm:

```bash
npm install ocdots
```

To use with node just import the module:

```javascript
const ocdots = require("ocdots");
ocdots.movePoints(...);
```

In the browser import the script and then

```html
<script src="node_modules/ocdots/ocdots.js"></script>
<script>
  movePoints(...);
</script>
```

You can also [download](https://raw.githubusercontent.com/luxedo/OCDots/master/ocdots.js)
the file and import as you prefer. No extra dependencies are required.

## Usage

Please check the functions signatures and description in the
[source](https://raw.githubusercontent.com/luxedo/OCDots/master/ocdots.js).

A running example is running [here](https://luxedo.github.io/OCDots/)

## License

> SPOS - Small Payload Object Serializer
> Copyright (C) 2020 Luiz Eduardo Amaral <luizamaral306@gmail.com>
>
> This program is free software: you can redistribute it and/or modify
> it under the terms of the GNU General Public License as published by
> the Free Software Foundation, either version 3 of the License, or
> (at your option) any later version.
>
> This program is distributed in the hope that it will be useful,
> but WITHOUT ANY WARRANTY; without even the implied warranty of
> MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
> GNU General Public License for more details.
>
> You should have received a copy of the GNU General Public License
