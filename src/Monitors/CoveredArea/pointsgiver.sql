SELECT load_extension('mod_spatialite')

ATTACH DATABASE '/home/nikolai/lststools/dune/misc/results.sqlite' AS res;

PRAGMA database_list

select astext(makeLine((select geom from res.tree where ID = t.ParentID), t.geom)) from res.tree as t limit 2

update coverage as s set weight = min( (weight+we*((50/0.5)/90)), 1) from (
select gid, 1/(1+exp(-(6.172439685021811 + dist*-0.033868111399770 ))) as we from (
select d.gid, dist from coverage, (
select gid, distance(geometry, geom) as dist from coverage,res.tree where dist < 600
) as d where d.gid = coverage.gid
)
) as sel where s.gid = sel.gid

/* For the lines, remember to change speed, cell size and model accordingly */
update coverage as s set weight = min( weight+uwes, 1) from (
select min(sum(wes), 1) as uwes, gid from (
select min( (we*((50/0.5)/90)), 1) as wes, gid from (
select gid, 1/(1+exp(-(6.172439685021811 + dist*-0.033868111399770 ))) as we, * from (
select d.gid, dist, * from coverage, (
select gid, distance(geometry, makeLine((select geom from tree where ID = t.ParentID), t.geom)) as dist from coverage, tree as t where dist < 600
) as d where d.gid = coverage.gid
)
) order by gid
) group by gid
) as sel where s.gid = sel.gid

/* For StationKeep Maneuvers */
update coverage as s set weight = min( weight+uwes, 1) from (
select min(sum(wes), 1) as uwes, gid from (
select min( (we*45/90), 1) as wes, gid from (
select gid, 1/(1+exp(-(4.748444238767068 + dist*-0.011050835615990 ))) as we, * from (
select d.gid, dist, * from coverage, (
select gid, distance(geometry,t.geom) as dist from coverage, tree as t where dist < 670
) as d where d.gid = coverage.gid
)
) order by gid
) group by gid

) as sel where s.gid = sel.gid


select makeline(geom) from tree
