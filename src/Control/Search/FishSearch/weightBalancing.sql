

-- To demonstrate weight balancing effect
	select gid,
		weight
		-0.001000*ABS( 0.000000-azimuth((select centroid(geometry) from FishSearch where gid = 2046), (select centroid(geometry) from FishSearch where gid = sg.gid)))
		-0.000050*distance((select centroid(geometry) from FishSearch where gid = 2046), (select centroid(geometry) from FishSearch where gid = sg.gid))
	 as wgt,
	weight
	,0.001000*ABS( 0.000000-azimuth((select centroid(geometry) from FishSearch where gid = 2046), (select centroid(geometry) from FishSearch where gid = sg.gid)))
	,0.000050*distance((select centroid(geometry) from FishSearch where gid = 2046), (select centroid(geometry) from FishSearch where gid = sg.gid))
	from FishSearch as sg where weight between -0.1 and 1.1

-- Alternative that makes all distances sum to one
    select gid, max(wgt) from (
	select gid,
		weight
		-0.001000*ABS( 0.000000-azimuth((select centroid(geometry) from FishSearch where gid = 2046), (select centroid(geometry) from FishSearch where gid = sg.gid)))
		-0.000050*distance((select centroid(geometry) from FishSearch where gid = 2046), (select centroid(geometry) from FishSearch where gid = sg.gid))/
			(select sum(draw) from (
				select distance((select centroid(geometry) from FishSearch where gid = 2046), (select centroid(geometry) from FishSearch where gid = sg.gid)) as draw
			from FishSearch as sg
			)) 
	as wgt
	from FishSearch as sg where weight between -0.1 and 1.1
)
-- All divided by max
select gid, max(wgt) from (
	select gid,
		weight/(select max(weight) from FishSearch)
		-0.001000*ABS( 0.000000-azimuth((select centroid(geometry) from FishSearch where gid = 239), (select centroid(geometry) from FishSearch where gid = sg.gid)))/
		    (select sum(araw) from (
				select ABS( 0.000000-azimuth((select centroid(geometry) from FishSearch where gid = 239), (select centroid(geometry) from FishSearch where gid = ag.gid))) as araw
			from FishSearch as ag
			))
		-0.000050*distance((select centroid(geometry) from FishSearch where gid = 239), (select centroid(geometry) from FishSearch where gid = sg.gid))/
			(select sum(draw) from (
				select distance((select centroid(geometry) from FishSearch where gid = 239), (select centroid(geometry) from FishSearch where gid = dg.gid)) as draw
			from FishSearch as dg
			)) 
	as wgt
	from FishSearch as sg
)

-- Using SSA to ensure angle is mapped to +-PI.  (https://stackoverflow.com/questions/1878907/how-can-i-find-the-smallest-difference-between-two-angles-around-a-point)
select gid, max(heu) from (

select *,w-0.001/pi()*(abs((a-floor(a/n)*n)-PI()))-d as heu, 0.001/pi()*(abs((a-floor(a/n)*n)-PI()))as ar from (
 	select geometry,gid,
	weight/(select sum(weight) from FishSearch) as w
	,( (pi()-azimuth((select centroid(geometry) from FishSearch where gid = 804), (select centroid(geometry) from FishSearch where gid = sg.gid))) + PI())
	as a
	,distance((select centroid(geometry) from FishSearch where gid = 804), (select centroid(geometry) from FishSearch where gid = sg.gid))/
	(select sum(draw) from (
	select distance((select centroid(geometry) from FishSearch where gid = 804), (select centroid(geometry) from FishSearch where gid = dg.gid)) as draw
	from FishSearch as dg
	))
  	as d,
 	2*PI() as n
 	from FishSearch as sg
)


)
