
-- Experiment on centroid pre calculation

CREATE TABLE "cfishsearch" (
	"gid"	INTEGER,
	"effort"	REAL,
	"weight"	REAL,
	"geometry"	POLYGON, 
    "center"	POINT,
	PRIMARY KEY("gid" AUTOINCREMENT)
)

insert into cfishsearch select *, centroid(geometry) as center from FishSearch;

SELECT RecoverGeometryColumn('cfishsearch', 'center', 32632, 'POINT', 'XY')

select createSpatialIndex('cfishsearch','center')

select gid, max(w-0.000100/pi()*(abs((a-floor(a/2*PI())*2*PI())-PI()))-1.000000*d) from (
	select center,gid,weight/(select max(weight) from cFishSearch) as w
	,( (0.000000-azimuth((
			select center from cFishSearch where gid = 4220), (
			select center from cFishSearch where gid = sg.gid)
		)) + PI()) as a
	,distance((
			select center from cFishSearch where gid = 4220), (
			select center from cFishSearch where gid = sg.gid))/(
		select max(draw) from (
			select distance(
				(select center from cFishSearch where gid = 4220), (select center from cFishSearch where gid = dg.gid)) as draw from cFishSearch as dg
		) )as d
	from cFishSearch as sg where w>0.0
)
