

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