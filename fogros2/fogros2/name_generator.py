# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Word list from https://github.com/imsky/wordlists
#
# The MIT License (MIT)
#
# Copyright (c) 2017-2021 Ivan Malopinsky
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import random

_adjectives = [
"absolute",
"abstract",
"accepting",
"achromatic",
"acidic",
"acoustic",
"active",
"acute",
"acyclic",
"adagio",
"adaptive",
"adiabatic",
"adjacent",
"advanced",
"adventurous",
"affable",
"afraid",
"aged",
"agile",
"all",
"allegro",
"alternate",
"alternating",
"amber",
"ambitious",
"amiable",
"amicable",
"amortized",
"ancient",
"andante",
"angry",
"animato",
"annoying",
"another",
"antique",
"approximate",
"aquamarine",
"archaic",
"arctic",
"arid",
"ascent",
"ash",
"asphalt",
"associative",
"asymptotic",
"atomic",
"auburn",
"augmenting",
"average",
"avocado",
"azure",
"baked",
"balanced",
"bare",
"basic",
"beige",
"bent",
"best",
"beveled",
"big",
"binary",
"binding",
"bipartite",
"bisque",
"bitter",
"black",
"bland",
"blaring",
"blazing",
"blended",
"blistering",
"blocking",
"blocky",
"blue",
"bold",
"bone",
"boolean",
"booming",
"bordeaux",
"bounded",
"boxy",
"braised",
"brass",
"brave",
"breezy",
"bright",
"brilliant",
"briny",
"broad",
"bronze",
"brown",
"brownian",
"brutal",
"brute",
"brute force",
"buoyant",
"burgundy",
"burning",
"buttery",
"callous",
"calm",
"camel",
"canary",
"candied",
"caramel",
"caramelized",
"careful",
"cautious",
"celeste",
"central",
"cerulean",
"chalky",
"chamfered",
"champagne",
"charcoal",
"charitable",
"chartreuse",
"cheerful",
"cheesy",
"chestnut",
"chewy",
"chief",
"chill",
"chilly",
"chocolate",
"chocolaty",
"chromatic",
"chunky",
"citric",
"citron",
"claret",
"clean",
"clear",
"clever",
"closed",
"cloudy",
"cloying",
"coal",
"cobalt",
"coffee",
"coherent",
"coincident",
"cold",
"collinear",
"colorful",
"commutative",
"compact",
"complete",
"complex",
"complicated",
"composite",
"concave",
"concentric",
"concrete",
"concurrent",
"congruent",
"connected",
"constant",
"contemporary",
"convex",
"convoluted",
"cool",
"coped",
"coplanar",
"coral",
"corn",
"corporate",
"counting",
"courtly",
"covering",
"crabby",
"crazy",
"cream",
"creamy",
"creative",
"crimson",
"crispy",
"critical",
"cross",
"cruel",
"crunchy",
"curious",
"current",
"customer",
"cyan",
"cyclic",
"damaged",
"damp",
"daring",
"dark",
"deafening",
"decidable",
"delicious",
"denim",
"dense",
"descent",
"descriptive",
"desert",
"deterministic",
"devout",
"diachronic",
"diagonal",
"dichotomic",
"direct",
"distinct",
"district",
"doughy",
"drab",
"dry",
"dull",
"dyadic",
"dynamic",
"each",
"eager",
"easy",
"ebony",
"ecru",
"edible",
"either",
"elaborate",
"electric",
"electrical",
"elegant",
"emerald",
"endothermic",
"energetic",
"equidistant",
"equilateral",
"espressivo",
"every",
"exact",
"excited",
"exhaustive",
"exothermic",
"exponential",
"express",
"extended",
"external",
"extremal",
"factorial",
"faded",
"faint",
"fancy",
"fast",
"fat",
"feasible",
"feldspar",
"felt",
"fermented",
"ferocious",
"few",
"figurative",
"finite",
"fixed",
"flashed",
"flat",
"flavorful",
"flush",
"foggy",
"forgiving",
"formal",
"forward",
"frayed",
"free",
"freezing",
"fresh",
"fried",
"friendly",
"frigid",
"frosty",
"frozen",
"fuchsia",
"full",
"fundamental",
"funny",
"furious",
"future",
"generative",
"generous",
"genteel",
"gentle",
"giant",
"gilded",
"glad",
"glass",
"global",
"glossy",
"glowing",
"glum",
"gold",
"golden",
"gourmet",
"graceful",
"grandioso",
"grating",
"grave",
"gravitational",
"gray",
"greasy",
"great",
"greedy",
"green",
"grilled",
"grim",
"gritty",
"grizzled",
"grouchy",
"happy",
"hard",
"hasty",
"heartless",
"heather",
"helpful",
"hidden",
"hoary",
"honest",
"horizontal",
"hot",
"huge",
"human",
"humane",
"humble",
"humid",
"humongous",
"hushed",
"icy",
"immature",
"immediate",
"immense",
"impulsive",
"inclusive",
"independent",
"indigo",
"indulgent",
"inflammable",
"inscribed",
"instant",
"insulated",
"intense",
"interior",
"internal",
"international",
"intractable",
"intricate",
"inventive",
"inverse",
"inverted",
"investor",
"iron",
"irregular",
"isobaric",
"isochoric",
"isometric",
"isomorphic",
"isothermal",
"ivory",
"jellied",
"jet",
"joint",
"jolly",
"jovial",
"juicy",
"jumbo",
"juvenile",
"khaki",
"kind",
"kinetic",
"knurled",
"laminated",
"large",
"largo",
"late",
"latent",
"lazy",
"lead",
"lean",
"leather",
"legacy",
"legato",
"lenient",
"level",
"libretto",
"light",
"lime",
"linear",
"little",
"lively",
"livid",
"local",
"long",
"loud",
"lower",
"loyal",
"mad",
"magenta",
"magnetic",
"many",
"marinated",
"marked",
"maroon",
"mashed",
"massive",
"matching",
"matte",
"mature",
"maximum",
"mean",
"mechanical",
"median",
"medium",
"meek",
"merciless",
"merry",
"messy",
"metal",
"metallic",
"mild",
"milky",
"miniature",
"minimum",
"mint",
"minty",
"mode",
"moderato",
"modern",
"moist",
"molten",
"molto",
"muffled",
"mute",
"muted",
"naive",
"narrow",
"national",
"natural",
"navy",
"nearest",
"neat",
"nervous",
"new",
"nice",
"nimble",
"nippy",
"noisy",
"nondeterministic",
"novel",
"nuclear",
"null",
"nullary",
"numerous",
"nutritious",
"nutty",
"objective",
"obliging",
"oblique",
"obnoxious",
"obsolete",
"obtuse",
"obvious",
"odious",
"offline",
"oily",
"old",
"olive",
"one",
"online",
"open",
"optical",
"optimal",
"orange",
"ordered",
"organic",
"oriented",
"ornery",
"orthogonal",
"oscillating",
"other",
"overcast",
"pale",
"pallid",
"paper",
"parallel",
"partial",
"patient",
"perfect",
"perpendicular",
"persistent",
"piercing",
"pink",
"piquant",
"pizzicato",
"plain",
"planar",
"plastic",
"pleasant",
"plum",
"plumb",
"poached",
"pointed",
"poky",
"polite",
"polynomial",
"potential",
"pounded",
"prepared",
"presto",
"primary",
"primordial",
"principal",
"product",
"progressive",
"prompt",
"proper",
"proud",
"pure",
"pureed",
"purple",
"quadratic",
"quantum",
"quick",
"quiet",
"radial",
"radiant",
"radioactive",
"ragged",
"rainy",
"rancid",
"random",
"randomized",
"rank",
"rapid",
"raw",
"recent",
"rectilinear",
"recursive",
"red",
"reduced",
"refined",
"regional",
"regular",
"relative",
"relaxed",
"religious",
"resolving",
"resonnt",
"respectful",
"resultant",
"rich",
"right",
"rigid",
"ripe",
"roaring",
"rosy",
"round",
"rounded",
"rowdy",
"rubbery",
"rude",
"rust",
"sad",
"salmon",
"salty",
"saucy",
"savage",
"savory",
"scalding",
"scared",
"searing",
"seasoned",
"seething",
"selfish",
"senile",
"senior",
"sensitive",
"serious",
"several",
"shabby",
"sharp",
"shiny",
"short",
"shortest",
"shrewd",
"shy",
"sienna",
"silent",
"silly",
"silver",
"similar",
"simmered",
"simple",
"sizzling",
"skinny",
"sleek",
"slim",
"slow",
"sluggish",
"small",
"smart",
"smoggy",
"smoked",
"smoky",
"snow",
"snowy",
"soft",
"solid",
"soluble",
"some",
"sophisticated",
"sour",
"sparse",
"spatial",
"speedy",
"spicy",
"spry",
"square",
"stable",
"staccato",
"staff",
"stale",
"steamed",
"steel",
"stern",
"sticky",
"stone",
"straight",
"stringy",
"strong",
"stubborn",
"succulent",
"sugary",
"sunny",
"swarm",
"sweet",
"swift",
"symmetric",
"synchronic",
"syrupy",
"tall",
"tan",
"tangy",
"tart",
"tattered",
"taxonomic",
"teal",
"tempered",
"tender",
"tense",
"terminal",
"ternary",
"thick",
"thin",
"threadbare",
"threaded",
"thundering",
"timid",
"tiny",
"toasted",
"tomato",
"topped",
"tossed",
"tough",
"tractable",
"tranquil",
"trite",
"trusting",
"unary",
"undecidable",
"undirected",
"uniform",
"universal",
"unproductive",
"unsolvable",
"unsorted",
"upbeat",
"urbane",
"vain",
"vertical",
"vibrant",
"vibrato",
"vicious",
"vintage",
"violent",
"violet",
"visible",
"vivid",
"volumetric",
"wan",
"warm",
"warped",
"wary",
"watery",
"weary",
"weathered",
"weighted",
"white",
"wicker",
"wide",
"windy",
"wintry",
"wise",
"witty",
"wood",
"wooden",
"woolen",
"worn",
"worried",
"yellow",
"young",
"yummy",
"zesty",
"zingy",
]
_nouns = [
"abbey",
"aberration",
"abscissa",
"absorption",
"acacia",
"acceleration",
"accelerator",
"accent",
"accident",
"account",
"accrual",
"accuracy",
"acid",
"acquisition",
"acre",
"action",
"activation",
"actor",
"actress",
"actuary",
"adaptation",
"adapter",
"adhesive",
"adjugate",
"adjuster",
"advance",
"advection",
"adventure",
"adversary",
"aerator",
"age",
"agent",
"aggregate",
"agreeance",
"agreement",
"airport",
"airwatt",
"airway",
"albariño",
"albatross",
"albedo",
"alert",
"alfalfa",
"algorithm",
"align",
"alignment",
"allegory",
"alley",
"alligator",
"alloy",
"alpha",
"alphabet",
"alternator",
"altitude",
"aluminum",
"amarone",
"amberjack",
"ambiance",
"ambient",
"ambush",
"amortization",
"amp",
"ampere",
"amplifier",
"analog",
"analogy",
"analyst",
"ancestor",
"anchovy",
"angel",
"angle",
"angler",
"angora",
"animation",
"anion",
"announcer",
"annuity",
"anode",
"antagonist",
"antenna",
"anthology",
"antiquity",
"apartment",
"aperture",
"apex",
"app",
"apparition",
"apple",
"application",
"appraisal",
"appreciation",
"apricot",
"apron",
"aqueduct",
"arbitration",
"arbor",
"arcade",
"arch",
"archer",
"archetype",
"archipelago",
"architect",
"architecture",
"archive",
"archway",
"area",
"arena",
"argument",
"armament",
"armory",
"arneis",
"aroma",
"arpeggio",
"arrangement",
"array",
"arrow",
"arrowroot",
"arroyo",
"artifact",
"artillery",
"asadero",
"ascender",
"ash",
"asiago",
"aside",
"asphalt",
"asset",
"assignment",
"assistant",
"associate",
"assumption",
"assurance",
"athlete",
"atlas",
"atmosphere",
"atoll",
"atom",
"attache",
"attachment",
"attenuation",
"audience",
"audio",
"audit",
"auditor",
"auslese",
"author",
"autobiography",
"automaton",
"auxiliary",
"availability",
"avenue",
"avocado",
"avocet",
"axe",
"axel",
"axis",
"ayu",
"azimuth",
"backburner",
"backlist",
"backpack",
"backstory",
"badger",
"badminton",
"bag",
"bagel",
"baggage",
"bagging",
"bailee",
"bakery",
"balance",
"balcony",
"balinese",
"ball",
"baluster",
"balustrade",
"bamboo",
"banana",
"band",
"bandit",
"bandwidth",
"bank",
"bankruptcy",
"banner",
"banquette",
"banylus",
"bar",
"barbaresco",
"barbel",
"barbette",
"bardolino",
"bargain",
"bark",
"barn",
"barnacles",
"barolo",
"barracks",
"barracuda",
"barrage",
"barrel",
"barrier",
"baryon",
"base",
"baseline",
"basement",
"basil",
"basin",
"basis",
"basket",
"bass",
"bassoon",
"bastion",
"bat",
"batch",
"batter",
"battery",
"bay",
"bayes",
"bayonet",
"bayou",
"bazaar",
"beagle",
"beaker",
"beam",
"bean",
"beans",
"bear",
"bearing",
"beat",
"beaujolais",
"bed",
"beech",
"beef",
"beer",
"bell",
"belt",
"belvedere",
"bench",
"bend",
"beneficiary",
"benefit",
"bengal",
"berm",
"berry",
"beta",
"betta",
"bevel",
"bezier",
"bias",
"bibliography",
"bicycle",
"bifocal",
"bight",
"bike",
"bilge",
"billboard",
"billet",
"bin",
"binary",
"bind",
"bingo",
"bintree",
"biography",
"biped",
"birch",
"bird",
"birman",
"birth",
"biscuit",
"bisector",
"bismuth",
"bison",
"bit",
"blade",
"blanc",
"bleed",
"blend",
"block",
"blockade",
"blower",
"blowfish",
"BLT",
"blue",
"blur",
"boa",
"board",
"boat",
"bobtail",
"bocaccio",
"bocce",
"body",
"bog",
"bogey",
"boilerplate",
"bold",
"bollard",
"bombay",
"bond",
"bone",
"bongo",
"book",
"bookcase",
"bookshelf",
"boom",
"boondoggle",
"booster",
"boosting",
"boot",
"booth",
"border",
"borzoi",
"bosun",
"bot",
"bottle",
"boulevard",
"bounce",
"bound",
"boundary",
"bourbon",
"boutique",
"bow",
"bowl",
"box",
"boxer",
"brace",
"bracket",
"brad",
"branch",
"brandy",
"brass",
"breach",
"bread",
"breakout",
"breezeway",
"brick",
"bridge",
"brie",
"briefcase",
"brig",
"bright",
"brightness",
"broadcast",
"broadcloth",
"broker",
"bronze",
"brook",
"broth",
"brownstone",
"browser",
"brush",
"buck",
"bucket",
"bud",
"budget",
"budgie",
"buffalo",
"buffer",
"buffet",
"bug",
"bugle",
"build",
"bulb",
"bulkhead",
"bulldozer",
"bullet",
"bulwark",
"bumper",
"bungalow",
"bunk",
"bunker",
"buoy",
"burbot",
"bureau",
"burger",
"burgundy",
"burl",
"burmese",
"burn",
"burrata",
"burrito",
"burst",
"bus",
"bush",
"butte",
"butter",
"butterkase",
"button",
"buyer",
"buy-in",
"buyout",
"byline",
"byte",
"cabana",
"cabernet",
"cabin",
"cabinet",
"cabriolet",
"cache",
"cactus",
"cadence",
"cadet",
"cafe",
"cage",
"cake",
"calendar",
"calico",
"caliper",
"callable",
"callback",
"calorie",
"camembert",
"camera",
"camp",
"can",
"canal",
"canary",
"candela",
"candy",
"canister",
"canity",
"cannon",
"canoe",
"canon",
"cantaloupe",
"canteen",
"canvas",
"canyon",
"cap",
"capacitor",
"capacity",
"cape",
"capital",
"capitalist",
"capitol",
"capstan",
"captain",
"caption",
"car",
"caramel",
"card",
"cardinality",
"caribou",
"carp",
"carriage",
"carrier",
"cart",
"carton",
"case",
"cash",
"casing",
"cask",
"casket",
"castle",
"catalyst",
"cataract",
"catch",
"category",
"cathedral",
"cathode",
"caulk",
"causeway",
"cave",
"caviar",
"cayenne",
"cedar",
"celesta",
"cell",
"cello",
"cellulose",
"cement",
"center",
"centerline",
"centroid",
"century",
"certificate",
"cevian",
"chablis",
"chain",
"chair",
"chaise",
"chalet",
"chambray",
"chamfer",
"champagne",
"change",
"changeset",
"channel",
"chapel",
"character",
"chardonnay",
"charge",
"charlie",
"chart",
"chateau",
"check",
"checkout",
"cheddar",
"cheese",
"cheeseburger",
"cheesesteak",
"cheetah",
"chenille",
"cherry",
"chest",
"chianti",
"chick",
"chickadee",
"chicken",
"chief",
"child",
"chili",
"chimichanga",
"chimpanzee",
"chino",
"chinon",
"chit",
"chives",
"chock",
"chocolate",
"chord",
"chow",
"chrome",
"chronology",
"church",
"chutney",
"cider",
"cilantro",
"cinema",
"cinnamon",
"circle",
"circuit",
"cistern",
"citadel",
"citron",
"citrus",
"city",
"civet",
"claim",
"clan",
"clapper",
"claret",
"clarinet",
"class",
"classic",
"classifier",
"clause",
"clearance",
"cleat",
"cleats",
"clef",
"clerk",
"cliche",
"click",
"client",
"cliff",
"climax",
"clip",
"clique",
"clone",
"cloth",
"cloud",
"clover",
"cloverleaf",
"club",
"clubhouse",
"cluster",
"clutch",
"clutter",
"coach",
"coastie",
"coating",
"cobbler",
"cobblestone",
"cobra",
"cockatiel",
"cockatoo",
"cockpit",
"cocoa",
"cocobolo",
"coconut",
"cod",
"coda",
"code",
"codec",
"coffee",
"coffer",
"cognac",
"colby",
"collateral",
"collie",
"collision",
"colonnade",
"color",
"column",
"combat",
"combination",
"combustion",
"command",
"commander",
"comment",
"commission",
"commit",
"community",
"compartment",
"compass",
"compete",
"complement",
"complexity",
"composer",
"compound",
"compression",
"compressor",
"comptroller",
"concentration",
"concept",
"concierge",
"concourse",
"concrete",
"condensation",
"condenser",
"condominium",
"conduction",
"conduit",
"cone",
"conference",
"configuration",
"confirmation",
"conflict",
"confusion",
"congruence",
"conjunction",
"consimilarity",
"console",
"constraint",
"consulate",
"consultant",
"container",
"content",
"continent",
"contingency",
"contour",
"contract",
"contrast",
"control",
"controller",
"convection",
"convergence",
"convert",
"converter",
"convertible",
"convolution",
"cookie",
"cooler",
"coopetition",
"coordinate",
"coordinator",
"coot",
"copper",
"copy",
"copyright",
"coral",
"cordon",
"corduroy",
"core",
"corgie",
"coriander",
"cork",
"cornice",
"corolla",
"corona",
"correlation",
"cosine",
"cost",
"costume",
"cotija",
"cottage",
"couch",
"cougar",
"coulomb",
"counterpoint",
"country",
"coupe",
"couple",
"course",
"courtyard",
"covariance",
"cove",
"cover",
"coverage",
"covey",
"cowboy",
"coyote",
"crankshaft",
"crate",
"cream",
"creative",
"credenza",
"credit",
"credits",
"creek",
"crescendo",
"crest",
"crew",
"cricket",
"crocodile",
"croissant",
"crop",
"crow",
"crowd",
"crumble",
"crypt",
"cube",
"cuckoo",
"cue",
"cuisine",
"culture",
"cume",
"cumin",
"cupola",
"curbbagel",
"curd",
"current",
"curry",
"curve",
"customer",
"cut",
"cycle",
"cyclist",
"cyclotron",
"cylinder",
"cymbal",
"dachshund",
"dado",
"daemon",
"dailies",
"dam",
"damask",
"damper",
"darby",
"dark",
"dart",
"darter",
"data",
"database",
"dataframe",
"dataset",
"date",
"datum",
"davit",
"deadhead",
"deadline",
"deal",
"death",
"debit",
"debt",
"debug",
"decay",
"decibel",
"decision",
"deck",
"decorator",
"deductible",
"deed",
"defense",
"defilade",
"degree",
"delay",
"deliverable",
"delta",
"demon",
"denim",
"denotation",
"denouement",
"density",
"department",
"deposition",
"depot",
"depth",
"deque",
"desert",
"design",
"designer",
"dessert",
"destination",
"detail",
"determinant",
"developer",
"diagonal",
"dialogue",
"diameter",
"diamond",
"diatonic",
"diction",
"dictionary",
"diff",
"differential",
"diffraction",
"diffuse",
"diffusion",
"digital",
"digraph",
"dill",
"dimension",
"dingo",
"dioptre",
"directive",
"director",
"directory",
"disconnect",
"discount",
"discriminant",
"discriminator",
"dish",
"disk",
"dispersion",
"displacement",
"dissonance",
"distance",
"distortion",
"dither",
"dive",
"diver",
"dividend",
"division",
"document",
"dodecagon",
"dog",
"dolce",
"dolcetto",
"dolly",
"domain",
"dome",
"dominant",
"donut",
"door",
"doorjamb",
"dope",
"dormer",
"dormitory",
"dot",
"double",
"dove",
"dowel",
"download",
"draft",
"dragster",
"drawbridge",
"drawer",
"drill",
"drink",
"drip",
"drive",
"driver",
"drivetrain",
"drone",
"drum",
"drywall",
"duck",
"duffel",
"dune",
"duplet",
"duplex",
"duration",
"dust",
"dynamic",
"eagle",
"easement",
"east",
"eaves",
"echelon",
"echo",
"edam",
"edge",
"edging",
"editor",
"editorial",
"eel",
"effect",
"effusion",
"eggs",
"elbow",
"electricity",
"electrolysis",
"electron",
"element",
"elevation",
"elk",
"ellipsis",
"em",
"embassy",
"embed",
"embedding",
"emboss",
"emerald",
"emmentaler",
"empanada",
"emporium",
"empowerment",
"emrasure",
"en",
"enamel",
"enchilada",
"encumbrance",
"energy",
"enfilade",
"engine",
"engineer",
"ensemble",
"ensign",
"entree",
"entrepreneur",
"entropy",
"envelope",
"epigram",
"epoch",
"equalizer",
"equator",
"equity",
"era",
"error",
"estimator",
"estuary",
"euphemism",
"event",
"exception",
"exchange",
"exec",
"executive",
"exercise",
"exit",
"expenditure",
"expense",
"experience",
"expertise",
"exponential",
"export",
"exposure",
"expression",
"extra",
"extraction",
"facade",
"face",
"facet",
"facilitator",
"factor",
"factorial",
"factory",
"fader",
"fair",
"falcon",
"fall",
"falloff",
"falls",
"family",
"fantail",
"farad",
"fare",
"fascia",
"father",
"fathom",
"faucet",
"feature",
"feed",
"feist",
"fen",
"fencing",
"fender",
"fennel",
"fermata",
"fern",
"feta",
"field",
"fig",
"filament",
"file",
"filet",
"filler",
"film",
"filter",
"finch",
"finger",
"fir",
"fire",
"firestop",
"firmware",
"fish",
"fitness",
"fitting",
"fixed",
"fixture",
"fjord",
"flag",
"flagstone",
"flam",
"flamingo",
"flank",
"flannel",
"flashing",
"flask",
"flat",
"fleece",
"fleet",
"flight",
"flitch",
"float",
"flora",
"flounder",
"flour",
"flow",
"flower",
"flue",
"fluid",
"fluke",
"flunky",
"flush",
"flute",
"flux",
"FM",
"focus",
"foie gras",
"folder",
"foley",
"folio",
"font",
"fontina",
"food",
"foot",
"footing",
"force",
"foreclosure",
"foremast",
"forest",
"fork",
"form",
"formant",
"format",
"formation",
"formatting",
"formula",
"fort",
"forte",
"fortress",
"fossa",
"fossil",
"founder",
"foundry",
"fowl",
"fox",
"fractal",
"frame",
"frank",
"freeway",
"freighter",
"frequency",
"fries",
"frieze",
"frisbee",
"frog",
"frogman",
"front",
"frontlist",
"fruit",
"fugue",
"function",
"functionality",
"funding",
"funnel",
"fur",
"fuse",
"fusion",
"futon",
"gabardine",
"gable",
"gaffer",
"gain",
"gallerie",
"gallery",
"galley",
"gallon",
"game",
"gang",
"garage",
"garden",
"garlic",
"gate",
"gatekeeper",
"gator",
"gauge",
"gazebo",
"gear",
"gearbox",
"generator",
"genre",
"geometry",
"geyser",
"ghost",
"ghoul",
"gig",
"gin",
"ginger",
"gingham",
"girder",
"git",
"gizmo",
"glacier",
"glass",
"glaze",
"glazing",
"glide",
"globe",
"gloss",
"glove",
"goal",
"goalie",
"goat",
"gofer",
"gold",
"goldbricker",
"goldfinch",
"golf",
"golfer",
"gong",
"goo",
"goose",
"gorgonzola",
"gothic",
"gouda",
"grade",
"gradient",
"grain",
"gram",
"granite",
"grant",
"grantor",
"grape",
"graph",
"graphite",
"grapnel",
"grappa",
"grass",
"gravel",
"gravity",
"grenade",
"grid",
"grill",
"grip",
"groove",
"group",
"grouper",
"grouse",
"grout",
"grove",
"growth",
"grunt",
"guarantor",
"guaranty",
"guard",
"guava",
"guide",
"guitar",
"gulch",
"gulf",
"gully",
"gum",
"gun",
"gunwale",
"gusset",
"gutter",
"gym",
"gymnast",
"haddock",
"hadron",
"haiku",
"hair",
"halbert",
"halftone",
"halibut",
"halite",
"hall",
"halo",
"hamburger",
"hand",
"hangar",
"harbor",
"hardball",
"hardcover",
"harp",
"harrier",
"hash",
"hatchback",
"havarti",
"hawk",
"head",
"headline",
"headset",
"heap",
"hearse",
"hearth",
"heat",
"heater",
"heather",
"heel",
"height",
"helm",
"helmet",
"helo",
"helper",
"henry",
"herb",
"heron",
"herring",
"hertz",
"hexagon",
"hickory",
"highlight",
"highline",
"highway",
"hill",
"himalayan",
"hint",
"hip",
"histogram",
"history",
"hoagie",
"hockey",
"holder",
"home",
"honey",
"hook",
"hoop",
"hoops",
"hop",
"horizon",
"horn",
"horror",
"horseradish",
"hospital",
"host",
"hostel",
"hot dog",
"hotel",
"hotrod",
"hound",
"hour",
"house",
"hub",
"HUD",
"hue",
"hull",
"human",
"humanist",
"humvee",
"husky",
"hut",
"hutch",
"hybrid",
"hydrolysis",
"hyperparameter",
"hypotenuse",
"hypothesis",
"ice",
"iceberg",
"idea",
"ideation",
"ident",
"identity",
"igloo",
"image",
"imperative",
"import",
"imprint",
"improvement",
"impulse",
"inch",
"incircle",
"incubator",
"indent",
"independence",
"index",
"inertia",
"infantry",
"inference",
"infield",
"inflation",
"inflection",
"influencer",
"infomediary",
"information",
"infrared",
"infrastructure",
"initiative",
"inlay",
"inlet",
"inline",
"inning",
"innovation",
"input",
"installation",
"instance",
"instructor",
"instrument",
"insulation",
"integer",
"integrator",
"intensity",
"interest",
"interface",
"interference",
"intersection",
"interval",
"inventory",
"inverse",
"inversion",
"investor",
"involute",
"ion",
"iron",
"island",
"islet",
"isotope",
"issue",
"italic",
"item",
"iteration",
"itinerary",
"ivy",
"jack",
"jackal",
"jackdaw",
"jacquard",
"jaguar",
"jail",
"jamb",
"jar",
"jarhead",
"javanese",
"javelin",
"jay",
"jeep",
"jersey",
"jetsam",
"jig",
"jigsaw",
"jingle",
"jog",
"joint",
"joist",
"joule",
"journal",
"judo",
"judy",
"jug",
"juice",
"jump",
"jumper",
"jungle",
"juniper",
"kapok",
"karate",
"kasseri",
"kayak",
"keel",
"keep",
"kelp",
"kerf",
"kern",
"kernel",
"kestrel",
"ketchup",
"kettle",
"key",
"keyframe",
"keyword",
"kicker",
"kilometer",
"kiosk",
"kitchen",
"kite",
"kiwi",
"knife",
"knot",
"komodo",
"korat",
"kudos",
"kudzu",
"laboratory",
"lacquer",
"lacrosse",
"ladder",
"lag",
"lagoon",
"lake",
"lamb",
"lambrusco",
"laminate",
"lamp",
"land",
"landaulet",
"landfall",
"landform",
"landing",
"landlubber",
"landscape",
"lane",
"language",
"lanyard",
"lard",
"lark",
"laser",
"latency",
"lath",
"latitude",
"lattice",
"layer",
"layout",
"layover",
"lead",
"leadership",
"leading",
"leadsman",
"leaf",
"league",
"learning",
"lease",
"leave",
"ledger",
"left",
"leg",
"legend",
"leisure",
"lemon",
"lens",
"leopard",
"lepton",
"lerp",
"levee",
"level",
"leverage",
"liability",
"liason",
"liberty",
"library",
"lien",
"life",
"lifer",
"ligature",
"light",
"lighthouse",
"lighting",
"lightship",
"lily",
"limburger",
"lime",
"limestone",
"limit",
"limiter",
"limo",
"limousine",
"line",
"liner",
"link",
"linkage",
"lintel",
"lion",
"liqueur",
"liquidation",
"liquor",
"list",
"literal",
"load",
"loader",
"loan",
"loch",
"locker",
"locus",
"lodge",
"loft",
"log",
"logic",
"logo",
"longhair",
"longitude",
"longshoreman",
"loon",
"loop",
"lose",
"loser",
"loss",
"lot",
"louver",
"loveseat",
"luge",
"luggage",
"lumber",
"lumen",
"luminosity",
"lynx",
"lyrics",
"macaw",
"mace",
"mackerel",
"madeira",
"magazine",
"magnification",
"magnitude",
"magpie",
"mahogany",
"mainmast",
"major",
"malbec",
"mall",
"mallet",
"maltese",
"mamba",
"manager",
"mandrill",
"mango",
"manifold",
"manor",
"mansion",
"mantel",
"manuscript",
"map",
"maple",
"marble",
"margarine",
"margin",
"marimba",
"marina",
"marinade",
"marker",
"market",
"marlin",
"marmalade",
"marsanne",
"marsh",
"mart",
"martin",
"mascarpone",
"maser",
"mask",
"mason",
"mass",
"mast",
"mastic",
"mat",
"mate",
"material",
"matrix",
"mayo",
"mayonnaise",
"mead",
"mean",
"meander",
"measure",
"meat",
"medal",
"media",
"median",
"melee",
"melody",
"melon",
"melt",
"member",
"membership",
"mercury",
"merge",
"meridian",
"merlot",
"mesa",
"mesh",
"message",
"metaball",
"metaphor",
"meter",
"method",
"methodology",
"metrics",
"metronome",
"microphone",
"midi",
"midpoint",
"mile",
"mileage",
"milk",
"mill",
"mindshare",
"mine",
"minivan",
"minor",
"mint",
"minute",
"mirror",
"miso",
"miter",
"mitt",
"mixture",
"moat",
"mobile",
"modal",
"mode",
"model",
"modem",
"modern",
"modulation",
"module",
"molality",
"molding",
"mole",
"molecule",
"moleskin",
"momentum",
"monastery",
"monitor",
"mono",
"monoid",
"monomer",
"monument",
"mortar",
"mortgage",
"moscato",
"mosque",
"moss",
"motel",
"motif",
"motion",
"motor",
"motorway",
"mountain",
"mousse",
"mouth",
"move",
"mozzarella",
"mud",
"muenster",
"muffin",
"muffler",
"mulberry",
"mullet",
"mullion",
"muntin",
"muscat",
"museum",
"mushroom",
"music",
"mustard",
"myth",
"mythology",
"nailer",
"name",
"napalm",
"narrative",
"narrator",
"narrows",
"nation",
"neck",
"nectar",
"nectarine",
"negation",
"nest",
"net",
"network",
"neuron",
"neutrino",
"neutron",
"newel",
"newton",
"niche",
"nickel",
"node",
"noir",
"noodle",
"norm",
"normal",
"north",
"nosing",
"notation",
"notch",
"note",
"novel",
"novella",
"nucleus",
"nugget",
"null",
"number",
"nut",
"nutmeg",
"oak",
"oasis",
"object",
"objective",
"oblique",
"oboe",
"observatory",
"occlusion",
"occurrence",
"ocean",
"ocelot",
"octagon",
"octave",
"octet",
"octree",
"offense",
"officer",
"offset",
"ohm",
"oil",
"olive",
"olympics",
"omega",
"omelette",
"onion",
"opacity",
"operator",
"opportunity",
"option",
"orange",
"orchestrator",
"order",
"ordnance",
"oregano",
"oriental",
"origin",
"originator",
"orphan",
"osprey",
"ostrich",
"ottoman",
"ounce",
"out",
"outboard",
"outlet",
"outlier",
"outline",
"outpost",
"output",
"outrigger",
"outwork",
"overhang",
"overpass",
"overtone",
"owl",
"owner",
"ownership",
"oxford",
"pace",
"packet",
"pad",
"paddle",
"page",
"pagoda",
"paint",
"pair",
"palace",
"palette",
"palisade",
"palm",
"pan",
"panel",
"panini",
"pantone",
"papaya",
"paper",
"paprika",
"parable",
"paradigm",
"parakeet",
"parallax",
"parallel",
"parameter",
"parapet",
"parent",
"parkway",
"parmesan",
"parody",
"parrot",
"parsley",
"participant",
"particle",
"partition",
"partnership",
"party",
"pascal",
"pass",
"passenger",
"passport",
"pasta",
"paste",
"pastry",
"patch",
"path",
"patio",
"pattern",
"pause",
"pavilion",
"payment",
"peach",
"peak",
"pear",
"pedestal",
"pediment",
"pelican",
"penguin",
"peninsula",
"penny",
"pentagon",
"pentatonic",
"pepato",
"pepper",
"perch",
"percussion",
"performance",
"pergola",
"perimeter",
"period",
"permutation",
"persian",
"pesto",
"petal",
"petrel",
"phalanx",
"phantasm",
"phantom",
"phaseacorn",
"pheasant",
"philly",
"phone",
"photometry",
"photon",
"phrase",
"piano",
"pica",
"piccolo",
"pie",
"pier",
"pigeon",
"pigment",
"pike",
"pilaster",
"pile",
"piles",
"pilot",
"pine",
"pineapple",
"pinot",
"pinscher",
"pinstripe",
"pint",
"pintail",
"pipeline",
"pique",
"pistol",
"pit",
"pita",
"pitch",
"pivot",
"pixel",
"pizza",
"plaid",
"plain",
"plan",
"plane",
"plank",
"planner",
"plant",
"plasma",
"plaster",
"plate",
"plateau",
"platform",
"platinum",
"play",
"playa",
"player",
"playlist",
"plisse",
"plot",
"plugin",
"plum",
"ply",
"plywood",
"pocket",
"pod",
"point",
"pointelle",
"pointer",
"points",
"polarization",
"pole",
"pollen",
"pollock",
"polo",
"poltergeist",
"polygon",
"polytope",
"pond",
"poodle",
"pool",
"poplar",
"poplin",
"popup",
"porch",
"pork",
"port",
"portal",
"porter",
"portico",
"portrait",
"poset",
"positron",
"post",
"poster",
"pot",
"potentiality",
"pouch",
"pouf",
"pound",
"poutine",
"power",
"powerset",
"practice",
"prairie",
"precision",
"predicate",
"preference",
"prefix",
"premise",
"preservative",
"pressure",
"pretzel",
"price",
"primer",
"print",
"printer",
"prior",
"prism",
"prison",
"procedure",
"process",
"producer",
"product",
"profile",
"program",
"programming",
"projectile",
"projection",
"promo",
"proof",
"prop",
"property",
"proposal",
"prosecco",
"protagonist",
"proton",
"provenance",
"provolone",
"prune",
"pseudonym",
"publicist",
"puck",
"pudding",
"puddle",
"puffin",
"pug",
"pulsar",
"pulse",
"pumice",
"pun",
"purchase",
"purlin",
"purse",
"pushback",
"putty",
"pyramid",
"python",
"quadtree",
"quail",
"quanta",
"quark",
"quart",
"quarter",
"quartz",
"quasar",
"quaver",
"quay",
"query",
"queso",
"quetzal",
"queue",
"quiver",
"rabbit",
"race",
"racer",
"rack",
"raclette",
"radar",
"radial",
"radian",
"radiation",
"radiator",
"radio",
"radiosity",
"radius",
"radix",
"radon",
"raft",
"rafter",
"ragdoll",
"ragged",
"raid",
"rail",
"raisin",
"rake",
"ramp",
"ranch",
"range",
"rank",
"rapids",
"rasp",
"raspberry",
"raster",
"rate",
"rating",
"rational",
"ravelin",
"raven",
"ravvet",
"ray",
"razorbill",
"reader",
"rebase",
"rebate",
"recall",
"receipt",
"recipe",
"recon",
"record",
"recordist",
"recourse",
"rectangle",
"rectory",
"recurrence",
"recursion",
"redoubt",
"redshift",
"reduction",
"reef",
"referee",
"refinery",
"reflection",
"refraction",
"refresh",
"refund",
"region",
"register",
"regression",
"rehab",
"relation",
"relationship",
"relativity",
"relaxation",
"relay",
"relish",
"reload",
"remote",
"remoulade",
"render",
"rent",
"repeat",
"repo",
"report",
"repository",
"representative",
"request",
"reservoir",
"residual",
"resin",
"resistivity",
"resolution",
"resonance",
"resource",
"rest",
"restaurant",
"result",
"resultant",
"retail",
"retreat",
"retriever",
"return",
"reuben",
"reveal",
"reverb",
"revolver",
"reward",
"rhea",
"rhombus",
"rhythm",
"ribbon",
"rice",
"ricotta",
"ride",
"ridge",
"riesling",
"riff",
"rifle",
"rig",
"right",
"ringtone",
"rink",
"rise",
"riser",
"river",
"rivulet",
"road",
"roadster",
"roadway",
"rocket",
"roll",
"roman",
"romano",
"roof",
"rook",
"root",
"rosé",
"rosemary",
"rosin",
"rotation",
"rotor",
"rottweiler",
"round",
"roundel",
"rout",
"router",
"row",
"rower",
"rowhouse",
"rubble",
"rum",
"run",
"rundown",
"runoff",
"runway",
"sack",
"saddle",
"sage",
"sail",
"salad",
"salamander",
"sale",
"salmon",
"salsa",
"salt",
"sample",
"sandcrab",
"sanding",
"sandstone",
"sandwich",
"sap",
"sapwood",
"sardine",
"sash",
"satchel",
"saturation",
"sauce",
"sauerkraut",
"sauvignon",
"sax",
"scab",
"scaffold",
"scale",
"scallop",
"scan",
"scarfing",
"scattering",
"scenario",
"scene",
"schema",
"schnauser",
"school",
"scope",
"score",
"scotch",
"scotia",
"screed",
"screen",
"screw",
"script",
"scrub",
"scuba",
"sea",
"sealer",
"search",
"seasoning",
"seat",
"secant",
"second",
"section",
"sectional",
"sedan",
"seed",
"segment",
"segno",
"segue",
"seller",
"semaphore",
"sempre",
"send",
"sensor",
"sequence",
"serif",
"serval",
"server",
"service",
"sesame",
"session",
"set",
"setback",
"settee",
"setter",
"shack",
"shackle",
"shader",
"shadow",
"shallows",
"shape",
"share",
"shares",
"shark",
"sharp",
"sheave",
"shed",
"sheet",
"shell",
"sherry",
"shift",
"shim",
"shine",
"ship",
"shiraz",
"shoal",
"shoot",
"shooter",
"shop",
"shore",
"shortcut",
"shot",
"shotgun",
"shoulder",
"shrike",
"shrine",
"shrink",
"shrub",
"siamese",
"siberian",
"sickbay",
"sidebar",
"sideboard",
"sidewalk",
"siding",
"siege",
"signature",
"sill",
"silo",
"silver",
"similarity",
"simulation",
"simulcast",
"singularity",
"sink",
"sitrep",
"skate",
"skeleton",
"skew",
"ski",
"skier",
"skill",
"skillet",
"skillset",
"skin",
"skipper",
"skirmish",
"skua",
"skunk",
"sky",
"skylark",
"skyscraper",
"skyway",
"slab",
"slalom",
"slate",
"sled",
"sledder",
"sleeper",
"slice",
"slide",
"slider",
"slope",
"slur",
"smoke",
"smooth",
"snapper",
"snare",
"sniffer",
"snipe",
"snippet",
"snowboard",
"soccer",
"soda",
"sofa",
"soil",
"soldier",
"sole",
"solenoid",
"solid",
"solo",
"solute",
"solution",
"solvent",
"sonar",
"sonata",
"sopapillas",
"sort",
"sortie",
"soul",
"sound",
"soup",
"source",
"south",
"soy",
"space",
"spam",
"span",
"spaniel",
"sparrow",
"speaker",
"spear",
"spec",
"specialist",
"specter",
"specular",
"speed",
"sphere",
"spice",
"spine",
"spire",
"spirit",
"spitz",
"spline",
"spook",
"spore",
"sport",
"spot",
"spread",
"spring",
"sprite",
"sprout",
"spruce",
"squad",
"square",
"squash",
"squib",
"squircle",
"sriracha",
"stable",
"stack",
"stadium",
"staff",
"stage",
"stager",
"staging",
"stain",
"stalk",
"starling",
"starter",
"startup",
"state",
"statement",
"static",
"station",
"status",
"stave",
"steak",
"steel",
"stem",
"step",
"stereo",
"stern",
"stew",
"stick",
"sting",
"stock",
"stockade",
"stool",
"store",
"stork",
"story",
"storyboard",
"strain",
"strait",
"strata",
"strategist",
"strategy",
"stream",
"stress",
"string",
"strip",
"structure",
"strut",
"stucco",
"stud",
"studio",
"stunt",
"style",
"stylus",
"sub",
"subcompact",
"subdivision",
"subfloor",
"submodule",
"subscriber",
"subset",
"subspace",
"suffix",
"sugar",
"suitcase",
"suite",
"sum",
"sump",
"supernova",
"superset",
"supervisor",
"support",
"surf",
"surface",
"surfer",
"surrender",
"surround",
"survey",
"suv",
"svn",
"swab",
"swale",
"swallow",
"swamp",
"swatch",
"sweep",
"sweeper",
"swell",
"swift",
"swim",
"swimmer",
"swing",
"swiss",
"switch",
"sword",
"sycamore",
"symbol",
"symmetry",
"symphony",
"sync",
"synergy",
"synopsis",
"syntax",
"synth",
"syrup",
"system",
"tab",
"tabby",
"table",
"taco",
"tactics",
"tag",
"tail",
"take",
"taking",
"talent",
"tambourine",
"tanager",
"tangent",
"tangerine",
"tank",
"tanker",
"tap",
"taper",
"taping",
"taquito",
"target",
"tarragon",
"tartar",
"taste",
"taxi",
"tea",
"teak",
"team",
"teapot",
"teaser",
"technician",
"technology",
"tee",
"template",
"temple",
"tempo",
"tenant",
"tenement",
"tennis",
"tensor",
"tent",
"tequila",
"term",
"terminal",
"terrace",
"terrier",
"territory",
"terry",
"tesla",
"tessellation",
"test",
"tetrad",
"text",
"texture",
"theater",
"theme",
"thinner",
"thorn",
"threshold",
"throw",
"thrush",
"thrust",
"thumbnail",
"thyme",
"ticket",
"tie",
"tiger",
"tilapia",
"timber",
"time",
"timer",
"timing",
"timpani",
"tin",
"tincan",
"tint",
"titanium",
"title",
"toast",
"tom",
"ton",
"tone",
"tongue",
"tonic",
"torpedo",
"torus",
"tostada",
"total",
"tote",
"toucan",
"touch",
"tournament",
"tower",
"townhouse",
"track",
"tracking",
"traffic",
"trainer",
"transfer",
"transform",
"transformation",
"transit",
"translation",
"transmission",
"transmitter",
"transom",
"transpose",
"trap",
"travel",
"tray",
"tread",
"treble",
"tree",
"trellis",
"trench",
"triad",
"triangle",
"triathlon",
"tributary",
"trie",
"trigger",
"trill",
"trim",
"trip",
"triplet",
"triplex",
"tritone",
"trombone",
"tropics",
"trough",
"trout",
"truck",
"trumpet",
"trunk",
"truss",
"trust",
"tub",
"tuba",
"tube",
"tuna",
"tundra",
"tungsten",
"tunnel",
"tuple",
"turbine",
"turkey",
"turmeric",
"turntable",
"turpentine",
"turret",
"twig",
"twill",
"twitch",
"type",
"umpire",
"underpass",
"underwriter",
"union",
"unit",
"universe",
"university",
"upload",
"upside",
"upstream",
"url",
"urn",
"use",
"user",
"valance",
"valid",
"valley",
"valuation",
"value",
"valve",
"van",
"variable",
"variance",
"varnish",
"vase",
"vault",
"veal",
"vector",
"vehicle",
"vein",
"velocity",
"velour",
"velvet",
"vendor",
"veneer",
"venison",
"venture",
"veranda",
"verbiage",
"version",
"vertex",
"viaduct",
"vialadministrator",
"vibes",
"vibration",
"vibrato",
"view",
"vignette",
"villa",
"vine",
"vinegar",
"viola",
"violin",
"viper",
"vision",
"vivaceaccelerometer",
"vocal",
"vocoder",
"vodka",
"voice",
"void",
"voile",
"volcano",
"volley",
"volt",
"voltage",
"volume",
"voxel",
"vulture",
"waffle",
"wagon",
"wake",
"walk",
"wall",
"wallpaper",
"walnut",
"warbler",
"wardrobe",
"warp",
"wasabi",
"wash",
"watch",
"watchtower",
"water",
"waterfall",
"watt",
"wattage",
"wave",
"wavelength",
"wax",
"weaponaltitude",
"web",
"weed",
"weight",
"west",
"westie",
"wetland",
"wharf",
"wheat",
"wheel",
"whippet",
"whisky",
"width",
"win",
"winch",
"window",
"wine",
"wingman",
"winner",
"winning",
"wire",
"wireframe",
"wireless",
"withdrawalanchor",
"wok",
"wolf",
"wolverine",
"woodamplifier",
"workampere",
"world",
"worldaccuracy",
"wraith",
"wrap",
"wrestler",
"writeradmin",
"xylophone",
"yak",
"yard",
"yardamplitude",
"yaw",
"yeast",
"yeomanagate",
"yield",
"yogurt",
"zinc",
"zinfandel",
"zipper",
"zoningact",
"zoomaccent",
]

def get_unique_name():
    r = random.SystemRandom()
    return f"{r.choice(_adjectives)}-{r.choice(_nouns)}"