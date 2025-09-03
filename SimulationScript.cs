using System;
using System.Collections.Generic;
using TMPro;
using Unity.Burst;
using Unity.Cinemachine;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

#region Primary Manager Scripts

/// <summary>
/// The primary simulation manager.
/// </summary>
public class SimulationManager : MonoBehaviour
{
    // --- VARIABLES ---
    // Singleton
    public static SimulationManager instance;
    public Bounds WorldBounds { get => _worldBounds.bounds; }
    [SerializeField] private Collider2D _worldBounds;

    [Header("Predator Settings")]
    public PredatorSettings PredatorSettings;

    [Header("Prey Settings")]
    public PreySettings PreySettings;

    [Header("Plant Settings")]
    public PlantSettings PlantSettings;

    [Header("Camera")]
    [SerializeField] private GameObject cameraControllerObject;
    [SerializeField] private CameraSettings cameraSettings;

    [Header("Simulation Time Manager")]
    [SerializeField] private GameObject simulationSpeedManagerObject;
    [SerializeField] private Slider simulationSpeedSlider;
    [SerializeField] private TMP_InputField simulationSpeedInputField;
    [SerializeField] private TMP_Text simulationSpeedDisplayText;

    [Header("Agent Info Display")]
    [SerializeField] private GameObject agentInfoDisplayManagerObject;
    [SerializeField] private TMP_Text agentTitleText;
    [SerializeField] private TMP_Text agentStateText;
    [SerializeField] private TMP_Text agentEnergyText;
    [SerializeField] private TMP_Text agentAgeText;
    [SerializeField] private TMP_Text agentGenderText;
    [SerializeField] private Image agentImage;
    [SerializeField] private GameObject infoPanelTransform;
    [SerializeField] private Transform agentHighlightTransform;

    // Internal References and Values
    private ObjectPoolManager objectPoolManager;


    // --- METHODS ----
    private void Awake()
    {
        if (instance == null) instance = this;
        else Destroy(gameObject);
    }

    private void Start()
    {
        SetUpSimulation();
    }

    private void OnDestroy() => instance = null;

    private void SetUpSimulation()
    {
        // Set up camera.
        if (cameraControllerObject != null)
        {
            CameraMovement cameraMovement = cameraControllerObject.AddComponent<CameraMovement>();
            cameraMovement.CameraData = cameraSettings;
        }
        else Debug.LogWarning("CameraControllerObject has not been assigned. Please assign it in the inspector.");

        // Set up simulation speed manager.
        if (simulationSpeedManagerObject != null)
        {
            SimulationSpeedManager simulationSpeedManager = simulationSpeedManagerObject.AddComponent<SimulationSpeedManager>();
            simulationSpeedManager.Initialise(simulationSpeedSlider, simulationSpeedDisplayText, simulationSpeedInputField);
        }
        else Debug.LogWarning("SimulationSpeedManagerObject has not been assigned. Please assign it in the inspector.");

        // Set up agent info display.
        if (agentInfoDisplayManagerObject != null)
        {
            AgentInfoDisplayManager agentInfoDisplayManager = agentInfoDisplayManagerObject.AddComponent<AgentInfoDisplayManager>();
            agentInfoDisplayManager.Initialise(
                agentTitleText,
                agentStateText,
                agentEnergyText,
                agentAgeText,
                agentGenderText,
                agentImage,
                infoPanelTransform,
                agentHighlightTransform
                );
        }
        else Debug.LogWarning("AgentInfoDisplayManagerObject has not been assigned. Please assign it in the inspector.");

        // Create object pools.
        InitialiseObjectPool();
    }

    private void InitialiseObjectPool()
    {
        // Create object pool manager.
        objectPoolManager = gameObject.AddComponent<ObjectPoolManager>();

        // Create entity initialisers with their corresponding agent types.
        Dictionary<Type, IEntityInitialiser> initialisers = new Dictionary<Type, IEntityInitialiser>
        {
            {typeof(PredatorSettings), new PredatorInitialiser()},
            {typeof(PreySettings), new PreyInitialiser() },
            { typeof(PlantSettings), new PlantInitialiser()}
        };

        // Set up object pools
        objectPoolManager.SetInitialisers(initialisers);
        objectPoolManager.CreateObjectPools(PredatorSettings, PreySettings, PlantSettings);
    }

    /// <summary>
    /// Reloads the current scene.
    /// </summary>
    public void RestartSimulation()
    {
        // Destroy all entities.
        EntityManager entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
        entityManager.DestroyEntity(entityManager.UniversalQuery);

        // Load the current scene.
        SceneManager.LoadSceneAsync(SceneManager.GetActiveScene().buildIndex);
    }

    /// <summary>
    /// Quits the game.
    /// </summary>
    public void QuitGame()
    {
        // Destroy all entities.
        EntityManager entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
        entityManager.DestroyEntity(entityManager.UniversalQuery);

        // Quit the game.
        Application.Quit();
    }
}

/// <summary>
/// Handles the object pools of <see cref="IObjectPool"/> objects. Can return objects to their associated pools or request a new object from a pool of the associated type.
/// </summary>
public class ObjectPoolManager : MonoBehaviour
{
    // --- VARIABLES ---
    // Singleton
    public static ObjectPoolManager instance;

    // Internal References and Values
    [Tooltip("Object Pool dictionary that stores a type as the key and the object pool as the value.")]
    private Dictionary<Type, Queue<IObjectPool>> agentPools;
    private Dictionary<Type, IEntityInitialiser> initialisers;
    private Bounds worldBounds;


    // --- METHODS ---
    private void Awake()
    {
        if (instance == null) instance = this;
        else Destroy(gameObject);
    }

    private void OnDestroy() => instance = null;

    /// <summary>
    /// Pass through a dictionary of agent types of their corresponding initialisers.
    /// </summary>
    public void SetInitialisers(Dictionary<Type, IEntityInitialiser> initialisers)
    {
        this.initialisers = initialisers;
    }

    /// <summary>
    /// Creates the object pools for each type of agent.
    /// </summary>
    /// <param name="allSpawnSettings">All the <see cref="SpawnSettings"/> for each agent type.</param>
    public void CreateObjectPools(params AgentSettings[] allSpawnSettings)
    {
        // Create new object pool dictionary and get world bounds.
        agentPools = new();
        worldBounds = SimulationManager.instance.WorldBounds;

        // Check that initialisers are valid.
        if (initialisers == null || initialisers.Count == 0)
        {
            Debug.LogError("Initialisers have not been set. Please call SetInitialisers first.");
            return;
        }

        // Get entity manager reference.
        EntityManager entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;

        // For each agent type, attempt to initialise it.
        foreach (AgentSettings settings in allSpawnSettings)
        {
            Type settingType = settings.GetType(); // Get settings type.

            // Check if has a reference to an initialiser matching the settings type.
            if (initialisers.TryGetValue(settingType, out IEntityInitialiser entityInitialiser))
            {
                // Create agents.
                List<IObjectPool> newAgents = entityInitialiser.Initialise(entityManager, settings);

                // Populate object pool.
                GenerateObjectPoolEntry(newAgents);

                // Initialise starting agents.
                if (typeof(EntityAgent).IsAssignableFrom(newAgents[0].GetType()))
                {
                    Type agentType = newAgents[0].GetType();

                    for (int i = 0; i < settings.SpawnSettings.InitialSpawnAmount; i++)
                    {
                        IObjectPool newAgent = RequestAgentFromPool(agentType);

                        if (newAgent != null && newAgent is EntityAgent agent)
                        {
                            newAgent.Initialise(GetRandomWorldLocation(), false);
                            entityManager.SetEnabled(agent.LinkedEntity, true);
                        }
                        else Debug.LogWarning($"{newAgent} is not an EntityAgent type and could not be initialised.");
                    }
                }
                else Debug.LogWarning($"{newAgents[0].GetType().Name} is not a valid agent type.");
            }
            else Debug.LogWarning($"No initialiser found for settings type: {settingType.Name}.");
        }
    }

    /// <summary>
    /// Gets a random location within the world boundaries.
    /// </summary>
    public Vector3 GetRandomWorldLocation()
    {
        float x = UnityEngine.Random.Range(worldBounds.min.x, worldBounds.max.x);
        float y = UnityEngine.Random.Range(worldBounds.min.y, worldBounds.max.y);

        return new Vector3(x, y, 0);
    }

    /// <summary>
    /// Returns an available agent (<see cref="IObjectPool"/>) from the associated object pool. 
    /// </summary>
    /// <param name="agentType">The type of agent to return.</param>
    public IObjectPool RequestAgentFromPool(Type agentType)
    {
        // Check if agent type is valid.
        if (agentPools.ContainsKey(agentType))
        {
            // Check if there are any available agents.
            if (agentPools[agentType].Count > 0)
            {
                // Get next available agent.
                IObjectPool newAgent = agentPools[agentType].Dequeue();
                if (newAgent == null) Debug.LogWarning($"Agent from pool of type: {agentType.Name} is NULL.");
                return newAgent;
            }
            else return null;
        }
        else
        {
            Debug.LogWarning($"Request Agent From Pool FAILED: There is no object pool for agent type: {agentType.Name}");
            return null;
        }
    }

    /// <summary>
    /// Returns an agent to it's associated object pool.
    /// </summary>
    public void ReturnAgentToObjectPool(IObjectPool agent)
    {
        // Check that agent is valid.
        if (agent == null)
        {
            Debug.LogWarning($"Attempted to return a NULL agent to object pool.");
            return;
        }

        Type agentType = agent.GetType(); // Get the type of the agent.

        // Check if agent type has an object pool.
        if (agentPools.TryGetValue(agentType, out Queue<IObjectPool> objectPool))
        {
            // Return agent into object pool.
            objectPool.Enqueue(agent);
        }
        else Debug.LogWarning($"Could not return agent of type {agentType.Name} to object pool: No object pool for the agent type.");
    }

    /// <summary>
    /// Generates a Queue and adds it to the appropriate object pool.
    /// </summary>
    /// <param name="agentList">The array of <see cref="IObjectPool"/> objects that should be added to the object pool.</param>
    private void GenerateObjectPoolEntry(List<IObjectPool> agentList)
    {
        if (agentList.Count == 0)
        {
            Debug.LogWarning($"Object array has a count of 0. Cannot create Queue Pool.");
            return;
        }

        // Store agent type.
        Type agentType = agentList[0].GetType();

        // Check if already exists in the object pool.
        if (!agentPools.ContainsKey(agentType))
        {
            // All all agents into a new queue.
            Queue<IObjectPool> queuePool = new();

            for (int i = 0; i < agentList.Count; i++)
            {
                queuePool.Enqueue(agentList[i]);
            }

            // Add the new queue to the object pool of the related agent type.
            agentPools.Add(agentType, queuePool);
        }
    }
}

/// <summary>
/// Helper class for initialising and settings up Components within ECS.
/// </summary>
public static class AgentComponentHelper
{
    // --- METHODS ---
    /// <summary>
    /// Sets all the common settings (settings shared by all agent types) to their default values.
    /// </summary>
    /// <param name="entityManager">The <see cref="EntityManager"/></param>
    /// <param name="entity">The entity with the components that should be set.</param>
    /// <param name="settings">The related <see cref="AgentSettings"/> for the entity that dictate the default values.</param>
    public static void SetCommonComponents(EntityManager entityManager, Entity entity, AgentSettings settings)
    {
        // Ageable
        entityManager.SetComponentData(entity, new AgeableComponent
        {
            // Maturity age plus variance.
            MaturityAge = settings.LifeExpectancySettings.MaturityAge + UnityEngine.Random.Range
            (-settings.LifeExpectancySettings.LifeExpectancyVariance,
                settings.LifeExpectancySettings.LifeExpectancyVariance),
            LifeExpectancy = settings.LifeExpectancySettings.LifeExpectancy,
            // Current age random up to half it's max age.
            CurrentAge = UnityEngine.Random.Range(0, settings.LifeExpectancySettings.LifeExpectancy / 2)
        });

        // Reproduce
        entityManager.SetComponentData(entity, new ReproduceComponent
        {
            ReproduceCooldown = settings.ReproductionSettings.ReproduceCooldown,
            Gender = ReproduceComponent.Genders.Undefined
        });

        // Timer
        entityManager.SetComponentData(entity, new TimerComponent { IsActive = false });

        // ReturnToPool
        entityManager.SetComponentEnabled<ReturnToPoolTag>(entity, false);
    }

    /// <summary>
    /// Creates a <see cref="GameObject"/>, adds an <see cref="EntityAgent"/> component to it and links the <see cref="Entity"/> and newly created <see cref="EntityAgent"/> together.
    /// </summary>
    /// <param name="entityManager">The <see cref="EntityManager"/></param>
    /// <param name="entity">The entity to create the object for.</param>
    /// <param name="settings">The related <see cref="AgentSettings"/>.</param>
    /// <param name="agentType">The type of <see cref="EntityAgent"/> that should be added to the newly created <see cref="GameObject"/>.</param>
    /// <returns></returns>
    public static IObjectPool CreateAndLinkGameObject(EntityManager entityManager, Entity entity, AgentSettings settings, Type agentType)
    {
        // Get random spawn position.
        Vector3 randomSpawnPosition;
        if (ObjectPoolManager.instance != null) randomSpawnPosition = ObjectPoolManager.instance.GetRandomWorldLocation();
        else randomSpawnPosition = Vector3.zero;

        // Create GameObject and add EntityAgent component.
        GameObject newGameObject = GameObject.Instantiate(settings.SpawnSettings.EntityPrefab, randomSpawnPosition, Quaternion.identity, settings.SpawnSettings.EntityParentContainer);
        if (!typeof(EntityAgent).IsAssignableFrom(agentType))
        {
            Debug.LogWarning($"{agentType.Name} is not a valid agent type.");
            return null;
        }
        EntityAgent entityAgent = newGameObject.AddComponent(agentType) as EntityAgent;

        // Check that newly created game object is valid.
        if (entityAgent == null)
        {
            Debug.LogWarning($"Failed to add component of type {agentType.Name} to the new GameObject.");
            GameObject.Destroy(newGameObject);
            return null;
        }

        // Link entity to new object.
        entityAgent.EntityManager = entityManager;
        entityAgent.LinkedEntity = entity;
        entityAgent.AgentDefaultSettings = settings;

        // Link new object to entity.
        entityManager.AddComponentData(entity, new LinkedGameObjectComponent { LinkedGameObject = newGameObject });

        // Set local transform component to new game object's transform.
        if (entityManager.HasComponent<LocalTransform>(entity))
        {
            entityManager.SetComponentData(entity, new LocalTransform
            {
                Position = newGameObject.transform.position,
                Rotation = newGameObject.transform.rotation,
                Scale = 1f
            });
        }

        // Set state to inactive.
        newGameObject.SetActive(false);
        entityManager.SetEnabled(entity, false);

        // Return newly created IObjectPool.
        if (entityAgent is IObjectPool agent)
        {
            return agent;
        }

        // If entityAgent is not valid, return null.
        return null;
    }
}

#endregion

#region Initialisers

/// <summary>
/// Creates all the starting <see cref="Entity"/> and initialises it with it's default values.
/// </summary>
public interface IEntityInitialiser
{
    /// <summary>
    /// The responsibility of this method is to create the entities and set their initial data.
    /// </summary>
    public List<IObjectPool> Initialise(EntityManager entityManager, AgentSettings settings);
}

/// <summary>
/// The <see cref="IEntityInitialiser"/> for Predator Agent Types.
/// </summary>
public class PredatorInitialiser : IEntityInitialiser
{
    public List<IObjectPool> Initialise(EntityManager entityManager, AgentSettings settings)
    {
        // Get world bounds.
        if (SimulationManager.instance == null) Debug.LogWarning($"PredatorInitialiser: Could not locate the SimulationManager instance. Could not get world bounds.");
        Bounds worldBounds = SimulationManager.instance.WorldBounds;

        // Get predator settings.
        PredatorSettings predatorSettings = settings as PredatorSettings;
        if (predatorSettings == null)
        {
            Debug.LogWarning($"Invalid settings type for PredatorInitialiser.");
            return null;
        }

        // Create archetype.
        EntityArchetype predatorArchetype = entityManager.CreateArchetype(
            typeof(LocalTransform),
            typeof(AgentTypeComponent),
            typeof(LinkedGameObjectComponent),
            typeof(AgeableComponent),
            typeof(ReproduceComponent),
            typeof(MovementComponent),
            typeof(TimerComponent),
            typeof(HungerComponent),
            typeof(EnergyComponent),
            typeof(ReturnToPoolTag)
            );

        // Create entities.
        int poolAmount = predatorSettings.SpawnSettings.ObjectPoolAmount;
        NativeArray<Entity> entities = new NativeArray<Entity>(poolAmount, Allocator.Temp);
        List<IObjectPool> agents = new();
        entityManager.CreateEntity(predatorArchetype, entities);

        for (int i = 0; i < entities.Length; i++)
        {
            Entity entity = entities[i];

            // Set common components initial values.
            AgentComponentHelper.SetCommonComponents(entityManager, entity, predatorSettings);

            // Set predator component initial values.
            entityManager.SetComponentData(entity, new MovementComponent
            {
                MovementDrag = predatorSettings.MovementSettings.MovementDrag,
                // Movement speed plus a variance.
                MovementSpeed = predatorSettings.MovementSettings.MovementSpeed + UnityEngine.Random.Range
                    (-predatorSettings.MovementSettings.SpeedVariance,
                    predatorSettings.MovementSettings.SpeedVariance),
                MinWorldBounds = worldBounds != null ? worldBounds.min : Vector3.zero,
                MaxWorldBounds = worldBounds != null ? worldBounds.max : Vector3.zero,
                ReevaluateInterval = predatorSettings.MovementSettings.ReevaluateInterval,
                DirectionChangeChance = predatorSettings.MovementSettings.DirectionChangeChance,
                IdleChance = predatorSettings.MovementSettings.IdleChance,
                ReevaluateTimeRemaining = predatorSettings.MovementSettings.ReevaluateInterval
            });
            entityManager.SetComponentData(entity, new EnergyComponent
            {
                // Max energy plus a variance.
                MaxEnergy = predatorSettings.EnergySettings.EnergyMax + UnityEngine.Random.Range
                (-predatorSettings.EnergySettings.EnergyVariance,
                    predatorSettings.EnergySettings.EnergyVariance),
                CurrentEnergy = predatorSettings.EnergySettings.EnergyMax,
                HighThreshold = predatorSettings.EnergySettings.HighEnergyThreshold,
                LowThreshold = predatorSettings.EnergySettings.LowEnergyThreshold
            });
            entityManager.SetComponentData(entity, new HungerComponent
            {
                PreferredFoodSource = predatorSettings.HungerSettings.PreferredFoodType
            });
            entityManager.SetComponentData(entity, new AgentTypeComponent
            {
                TypeOfAgent = AgentTypeComponent.AgentType.Predator
            });

            // Set random starting gender.
            ReproduceComponent reproduceComponent = entityManager.GetComponentData<ReproduceComponent>(entity);
            reproduceComponent.Gender = (ReproduceComponent.Genders)UnityEngine.Random.Range((int)1, (int)3);
            entityManager.SetComponentData(entity, reproduceComponent);

            // Set starting state to Wandering.
            entityManager.AddComponentData(entity, new WanderingStateTag());

            // Create and link to a GameObject.
            IObjectPool newAgent = AgentComponentHelper.CreateAndLinkGameObject(entityManager, entity, predatorSettings, typeof(Agent_Predator));
            if (newAgent != null) agents.Add(newAgent);
        }

        entities.Dispose();
        Debug.Log($"Initialised {poolAmount} Predators.");

        return agents;
    }
}

/// <summary>
/// The <see cref="IEntityInitialiser"/> for Prey Agent Types.
/// </summary>
public class PreyInitialiser : IEntityInitialiser
{
    public List<IObjectPool> Initialise(EntityManager entityManager, AgentSettings settings)
    {
        // Get world bounds.
        if (SimulationManager.instance == null) Debug.LogWarning($"PredatorInitialiser: Could not locate the SimulationManager instance. Could not get world bounds.");
        Bounds worldBounds = SimulationManager.instance.WorldBounds;

        // Get prey settings.
        PreySettings preySettings = settings as PreySettings;
        if (preySettings == null)
        {
            Debug.LogWarning($"Invalid settings type for PreyInitialiser.");
            return null;
        }

        // Create the archetype.
        EntityArchetype preyArchetype = entityManager.CreateArchetype(
            typeof(LocalTransform),
            typeof(AgentTypeComponent),
            typeof(LinkedGameObjectComponent),
            typeof(AgeableComponent),
            typeof(ReproduceComponent),
            typeof(MovementComponent),
            typeof(TimerComponent),
            typeof(HungerComponent),
            typeof(FoodSourceComponent),
            typeof(EnergyComponent),
            typeof(ReturnToPoolTag)
            );

        // Create entities.
        int poolAmount = preySettings.SpawnSettings.ObjectPoolAmount;
        NativeArray<Entity> entities = new NativeArray<Entity>(poolAmount, Allocator.Temp);
        List<IObjectPool> agents = new();
        entityManager.CreateEntity(preyArchetype, entities);

        for (int i = 0; i < entities.Length; i++)
        {
            Entity entity = entities[i];

            // Set common components initial values.
            AgentComponentHelper.SetCommonComponents(entityManager, entity, preySettings);

            // Set prey component initial values.
            entityManager.SetComponentData(entity, new MovementComponent
            {
                MovementDrag = preySettings.MovementSettings.MovementDrag,
                // Movement speed plus a variance.
                MovementSpeed = preySettings.MovementSettings.MovementSpeed + UnityEngine.Random.Range
                    (-preySettings.MovementSettings.SpeedVariance,
                    preySettings.MovementSettings.SpeedVariance),
                MinWorldBounds = worldBounds != null ? worldBounds.min : Vector3.zero,
                MaxWorldBounds = worldBounds != null ? worldBounds.max : Vector3.zero,
                ReevaluateInterval = preySettings.MovementSettings.ReevaluateInterval,
                DirectionChangeChance = preySettings.MovementSettings.DirectionChangeChance,
                IdleChance = preySettings.MovementSettings.IdleChance,
                ReevaluateTimeRemaining = preySettings.MovementSettings.ReevaluateInterval
            });
            entityManager.SetComponentData(entity, new EnergyComponent
            {
                // Max energy plus a variance.
                MaxEnergy = preySettings.EnergySettings.EnergyMax + UnityEngine.Random.Range
                (-preySettings.EnergySettings.EnergyVariance,
                    preySettings.EnergySettings.EnergyVariance),
                CurrentEnergy = preySettings.EnergySettings.EnergyMax,
                HighThreshold = preySettings.EnergySettings.HighEnergyThreshold,
                LowThreshold = preySettings.EnergySettings.LowEnergyThreshold
            });
            entityManager.SetComponentData(entity, new HungerComponent
            {
                PreferredFoodSource = preySettings.HungerSettings.PreferredFoodType
            });
            entityManager.SetComponentData(entity, new FoodSourceComponent
            {
                TypeOfFoodSource = preySettings.FoodSourceSettings.TypeOfFoodSource
            });
            entityManager.SetComponentData(entity, new AgentTypeComponent
            {
                TypeOfAgent = AgentTypeComponent.AgentType.Prey
            });

            // Set random starting gender.
            ReproduceComponent reproduceComponent = entityManager.GetComponentData<ReproduceComponent>(entity);
            reproduceComponent.Gender = (ReproduceComponent.Genders)UnityEngine.Random.Range((int)1, (int)3);
            entityManager.SetComponentData(entity, reproduceComponent);

            // Set starting state to Wandering.
            entityManager.AddComponentData(entity, new WanderingStateTag());

            // Create and link to a GameObject.
            IObjectPool newAgent = AgentComponentHelper.CreateAndLinkGameObject(entityManager, entity, preySettings, typeof(Agent_Prey));
            if (newAgent != null) agents.Add(newAgent);
        }

        entities.Dispose();
        Debug.Log($"Initialised {poolAmount} Prey.");

        return agents;
    }
}

/// <summary>
/// The <see cref="IEntityInitialiser"/> for Plant Agent Types.
/// </summary>
public class PlantInitialiser : IEntityInitialiser
{
    public List<IObjectPool> Initialise(EntityManager entityManager, AgentSettings settings)
    {
        // Get plant settings.
        PlantSettings plantSettings = settings as PlantSettings;
        if (plantSettings == null)
        {
            Debug.LogWarning($"Invalid settings type for PlantInitialiser.");
            return null;
        }

        // Set up archetype.
        EntityArchetype plantArchetype = entityManager.CreateArchetype(
            typeof(LocalTransform),
            typeof(AgentTypeComponent),
            typeof(LinkedGameObjectComponent),
            typeof(AgeableComponent),
            typeof(ReproduceComponent),
            typeof(TimerComponent),
            typeof(FoodSourceComponent),
            typeof(ReturnToPoolTag)
            );

        // Create entities.
        int poolAmount = plantSettings.SpawnSettings.ObjectPoolAmount;
        NativeArray<Entity> entities = new NativeArray<Entity>(poolAmount, Allocator.Temp);
        List<IObjectPool> agents = new();
        entityManager.CreateEntity(plantArchetype, entities);

        for (int i = 0; i < entities.Length; i++)
        {
            Entity entity = entities[i];

            // Set common components initial values.
            AgentComponentHelper.SetCommonComponents(entityManager, entity, plantSettings);

            // Set the plant components initial values.
            entityManager.SetComponentData(entity, new FoodSourceComponent
            {
                TypeOfFoodSource = plantSettings.FoodSourceSettings.TypeOfFoodSource
            });
            entityManager.SetComponentData(entity, new AgentTypeComponent
            {
                TypeOfAgent = AgentTypeComponent.AgentType.Plant
            });

            // Create and link to a GameObject.
            IObjectPool newAgent = AgentComponentHelper.CreateAndLinkGameObject(entityManager, entity, plantSettings, typeof(Agent_Plant));
            if (newAgent != null) agents.Add(newAgent);
        }

        entities.Dispose();
        Debug.Log($"Initialised {poolAmount} Plants.");

        return agents;
    }
}


#endregion

#region Agents

/// <summary>
/// The default class for all Agent Types.
/// </summary>
public class EntityAgent : MonoBehaviour, IObjectPool
{
    // --- VARIABLES ---
    /// <summary>
    /// The Entity that is linked to this Agent.
    /// </summary>
    public Entity LinkedEntity;
    /// <summary>
    /// The Entity Manager.
    /// </summary>
    public EntityManager EntityManager;
    /// <summary>
    /// The default AgentSettings for this Agent.
    /// </summary>
    public AgentSettings AgentDefaultSettings;


    // --- METHODS ---
    public void Initialise(Vector3 spawnPosition, bool resetData)
    {
        if (resetData) ResetAgent();

        // Set Agent's position to spawn position and activate it.
        gameObject.transform.position = spawnPosition;
        gameObject.SetActive(true);
    }

    private void LateUpdate()
    {
        if (World.DefaultGameObjectInjectionWorld == null || !World.DefaultGameObjectInjectionWorld.IsCreated ||
            LinkedEntity == null || EntityManager == null) return;

        // Perform additional methods set up by derived classes.
        AdditionalUpdateMethods();
    }

    public void ReturnToObjectPool()
    {
        // Deactivate Agent.
        gameObject.SetActive(false);

        // Check that Object Pool Manager is valid.
        if (ObjectPoolManager.instance == null)
        {
            Debug.LogWarning($"{gameObject.name} couldn't complete ReturnToObjectPool method: ObjectPoolManager instance is NULL.");
            return;
        }

        // Perform additional methods set up by derived classes.
        AdditionalReturnToObjectPoolMethods();

        // Return Agent to it's object pool.
        ObjectPoolManager.instance.ReturnAgentToObjectPool(this);
    }

    public void ResetAgent()
    {
        // Check that Entity Manager and World are all valid.
        if (World.DefaultGameObjectInjectionWorld == null || !World.DefaultGameObjectInjectionWorld.IsCreated ||
            LinkedEntity == null || EntityManager == null) return;

        // Reset age.
        AgeableComponent ageableComponent = EntityManager.GetComponentData<AgeableComponent>(LinkedEntity);
        ageableComponent.CurrentAge = 0f;
        EntityManager.SetComponentData(LinkedEntity, ageableComponent);

        // Reset reproduce values.
        ReproduceComponent reproduceComponent = EntityManager.GetComponentData<ReproduceComponent>(LinkedEntity);
        reproduceComponent.CanReproduce = false;
        EntityManager.SetComponentData(LinkedEntity, reproduceComponent);

        // Perform additional methods set up by derived classes.
        AdditionalResetMethods();
    }

    /// <summary>
    /// Intended to be overwritten by derived classes.
    /// </summary>
    protected virtual void AdditionalResetMethods() { }
    /// <summary>
    /// Intended to be overwritten by derived classes.
    /// </summary>
    protected virtual void AdditionalReturnToObjectPoolMethods() { }
    /// <summary>
    /// Intended to be overwritten by derived classes.
    /// </summary>
    protected virtual void AdditionalUpdateMethods() { }
}

/// <summary>
/// The <see cref="EntityAgent"/> class for the Predator Agent Type.
/// </summary>
public class Agent_Predator : EntityAgent
{
    // --- METHODS ---
    protected override void AdditionalUpdateMethods()
    {
        // Set Agent's position to it's linked entity's position.
        if (EntityManager.HasComponent<MovementComponent>(LinkedEntity))
        {
            transform.position = EntityManager.GetComponentData<LocalTransform>(LinkedEntity).Position;
        }
    }

    protected override void AdditionalResetMethods()
    {
        // Convert agent settings.
        PredatorSettings predatorSettings = AgentDefaultSettings as PredatorSettings;
        if (predatorSettings == null) return;

        // Reset energy.
        EnergyComponent energyComponent = EntityManager.GetComponentData<EnergyComponent>(LinkedEntity);
        energyComponent.CurrentEnergy = energyComponent.MaxEnergy;
        EntityManager.SetComponentData(LinkedEntity, energyComponent);
    }
}

/// <summary>
/// The <see cref="EntityAgent"/> class for the Prey Agent Type.
/// </summary>
public class Agent_Prey : EntityAgent
{
    // --- METHODS ---
    protected override void AdditionalUpdateMethods()
    {
        // Set Agent's position to it's linked entity's position.
        if (EntityManager.HasComponent<MovementComponent>(LinkedEntity))
        {
            transform.position = EntityManager.GetComponentData<LocalTransform>(LinkedEntity).Position;
        }
    }

    protected override void AdditionalResetMethods()
    {
        // Convert agent settings.
        PreySettings preySettings = AgentDefaultSettings as PreySettings;
        if (preySettings == null) return;

        // Reset energy.
        EnergyComponent energyComponent = EntityManager.GetComponentData<EnergyComponent>(LinkedEntity);
        energyComponent.CurrentEnergy = energyComponent.MaxEnergy;
        EntityManager.SetComponentData(LinkedEntity, energyComponent);
    }
}

/// <summary>
/// The <see cref="EntityAgent"/> class for the Plant Agent Type.
/// </summary>
public class Agent_Plant : EntityAgent { }

#endregion

#region Interfaces

/// <summary>
/// Attached to <see cref="EntityAgent"/>s that are used within an Object Pool.
/// </summary>
public interface IObjectPool
{
    /// <summary>
    /// Called when this agent is initialised.
    /// </summary>
    /// <param name="spawnPosition">The position it should be spawned to.</param>
    /// <param name="resetData">Whether or not the data should be reset.</param>
    public void Initialise(Vector3 spawnPosition, bool resetData);
    /// <summary>
    /// Returns this agent to it's associated object pool.
    /// </summary>
    public void ReturnToObjectPool();
    /// <summary>
    /// Resets the agent to a default, or starting, state.
    /// </summary>
    public void ResetAgent();
}

#endregion

#region Jobs

/// <summary>
/// Entity job that cycles through all <see cref="FoodSourceComponent.FoodSourceType"/> entities and locates the nearest one to each entity that is looking for food.
/// </summary>
[BurstCompile]
[WithNone(typeof(FoodFoundStateTag), typeof(MateFoundStateTag), typeof(AttemptingToReproduceTag))]
public partial struct FindNearestFoodJob : IJobEntity
{
    /// <summary>
    /// The array containing all food source data.
    /// </summary>
    [ReadOnly] public NativeArray<FoodData> FoodArray;
    public EntityCommandBuffer.ParallelWriter ECB;

    void Execute([EntityIndexInQuery] int sortKey, Entity entity, in EnergyComponent energy, in HungerComponent hunger, in LocalTransform transform)
    {
        // Check if energy is above Low Threshold.
        if (energy.CurrentEnergy > energy.LowThreshold) return;

        // Set default search values.
        Entity nearestFoodEntity = Entity.Null;
        float nearestDistanceSq = float.PositiveInfinity;

        // Loop through all food sources and find the nearest to entity.
        foreach (var foodData in FoodArray)
        {
            // Check if food source is of type PreferredFoodSource.
            if (foodData.TypeOfFoodSource != hunger.PreferredFoodSource) continue;

            // Check distance to food source.
            float distanceSq = math.distancesq(transform.Position, foodData.Position);
            if (distanceSq < nearestDistanceSq)
            {
                nearestDistanceSq = distanceSq;
                nearestFoodEntity = foodData.Entity;
            }
        }

        // Check if found a food source.
        if (nearestFoodEntity != Entity.Null)
        {
            ECB.RemoveComponent<WanderingStateTag>(sortKey, entity);
            ECB.AddComponent<FoodFoundStateTag>(sortKey, entity);
            ECB.AddComponent(sortKey, entity, new TargetFoodComponent { FoodTarget = nearestFoodEntity });
        }
    }
}

/// <summary>
/// Entity job that populates a <see cref="NativeArray{T}"/> with all <see cref="FoodSourceComponent.FoodSourceType"/> entities.
/// </summary>
[BurstCompile]
public partial struct PopulateFoodArrayJob : IJobEntity
{
    [WriteOnly] public NativeArray<FoodData> FoodArray;

    void Execute([EntityIndexInQuery] int index, Entity entity, in FoodSourceComponent food, in LocalTransform transform)
    {
        FoodArray[index] = new FoodData
        {
            Entity = entity,
            Position = transform.Position,
            TypeOfFoodSource = food.TypeOfFoodSource
        };
    }
}

/// <summary>
/// Entity job that cycles through all potential mate entities and locates the nearest one to each entity that is ready to reproduce.
/// </summary>
[BurstCompile]
[WithNone(typeof(FoodFoundStateTag), typeof(MateFoundStateTag), typeof(AttemptingToReproduceTag))]
public partial struct FindNearestMateJob : IJobEntity
{
    /// <summary>
    /// The array containing all mate source data.
    /// </summary>
    [ReadOnly] public NativeArray<MateData> MateArray;
    public EntityCommandBuffer.ParallelWriter ECB;

    void Execute([EntityIndexInQuery] int sortKey, Entity entity, in EnergyComponent energy, in ReproduceComponent reproduce, in LocalTransform transform, in AgentTypeComponent agentType)
    {
        // Check if entity is ready to reproduce and if energy is lower than High Threshold.
        if (!reproduce.ReadyToReproduce || energy.CurrentEnergy <= energy.HighThreshold) return;

        // Set default search values.
        Entity nearestMateEntity = Entity.Null;
        float nearestDistanceSq = float.PositiveInfinity;

        // Loop through all mates and find the nearest to entity.
        foreach (var mateData in MateArray)
        {
            // Check if mate is this entity.
            if (mateData.Entity == entity) continue;

            // Check if mate is valid for this entity.
            if (mateData.Gender == reproduce.Gender || !mateData.CanReproduce || mateData.AgentType != agentType.TypeOfAgent) continue;

            // Check distance to mate.
            float distanceSq = math.distancesq(transform.Position, mateData.Position);
            if (distanceSq < nearestDistanceSq)
            {
                nearestDistanceSq = distanceSq;
                nearestMateEntity = mateData.Entity;
            }
        }

        // Check if found a mate.
        if (nearestMateEntity != Entity.Null)
        {
            // Only initiate if index is lower than target mate's.
            if (entity.Index > nearestMateEntity.Index) return;

            // Stop wandering
            ECB.RemoveComponent<WanderingStateTag>(sortKey, entity);

            // Set state to Mate Found.
            ECB.AddComponent<MateFoundStateTag>(sortKey, entity);
            ECB.AddComponent(sortKey, entity, new TargetMateComponent { MateTarget = nearestMateEntity });

            // Stop mate from wandering and have them target this entity.
            ECB.RemoveComponent<WanderingStateTag>(sortKey, nearestMateEntity);
            ECB.AddComponent<MateFoundStateTag>(sortKey, nearestMateEntity);
            ECB.AddComponent(sortKey, nearestMateEntity, new TargetMateComponent { MateTarget = entity });
        }
    }
}

/// <summary>
/// Entity job that populates a <see cref="NativeArray{T}"/> with all potential mate entities.
/// </summary>
[BurstCompile]
public partial struct PopulateMateArrayJob : IJobEntity
{
    [WriteOnly] public NativeArray<MateData> MateArray;

    void Execute([EntityIndexInQuery] int index, Entity entity, in ReproduceComponent reproduce, in LocalTransform transform, in AgentTypeComponent agent)
    {
        MateArray[index] = new MateData
        {
            Entity = entity,
            Position = transform.Position,
            Gender = reproduce.Gender,
            CanReproduce = reproduce.CanReproduce,
            AgentType = agent.TypeOfAgent
        };
    }
}

#endregion

#region Systems

/// <summary>
/// Handles tracking if an entity can reproduce and if it's ready to.
/// </summary>
[BurstCompile]
public partial struct ReproduceSystem : ISystem
{
    public void OnUpdate(ref SystemState state)
    {
        foreach (var (reproduce, ageable, timer) in SystemAPI.Query<RefRW<ReproduceComponent>, RefRO<AgeableComponent>, RefRW<TimerComponent>>())
        {
            // If entity has not reached maturity, continue.
            if (!ageable.ValueRO.ReachedMaturity) continue;

            // Check if can reproduce.
            if (timer.ValueRO.TypeOfTimer != TimerComponent.TimerType.Reproduce || timer.ValueRO.IsActive) continue;

            // If can't reproduce but the timer has runout (is not active), set CanReproduce to true.
            if (!reproduce.ValueRO.CanReproduce)
            {
                reproduce.ValueRW.CanReproduce = true;
                reproduce.ValueRW.ReadyToReproduce = true;
            }
            // Check if no longer ready to reproduce (have just reproduced).
            else if (!reproduce.ValueRO.ReadyToReproduce)
            {
                reproduce.ValueRW.CanReproduce = false;

                // Start cooldown timer.
                timer.ValueRW.TypeOfTimer = TimerComponent.TimerType.Reproduce;
                timer.ValueRW.RemainingTime = reproduce.ValueRO.ReproduceCooldown;
                timer.ValueRW.IsActive = true;
            }
        }
    }
}

/// <summary>
/// Handles aging entities.
/// </summary>
[BurstCompile]
public partial struct AgingSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<AgeableComponent>();
    }

    public void OnUpdate(ref SystemState state)
    {
        foreach (var (ageable, reproduce, entity) in SystemAPI.Query<RefRW<AgeableComponent>, RefRW<ReproduceComponent>>().WithNone<ReturnToPoolTag>().WithEntityAccess())
        {
            // Age entity.
            ageable.ValueRW.CurrentAge += (SimulationSpeedManager.SimulationSpeed / 60);

            // Check if reached life expectancy.
            if (ageable.ValueRO.CurrentAge >= ageable.ValueRO.LifeExpectancy)
            {
                // Mark for death
                if (state.EntityManager.HasComponent<ReturnToPoolTag>(entity))
                {
                    state.EntityManager.SetComponentEnabled<ReturnToPoolTag>(entity, true);
                    continue;
                }
                Debug.LogWarning($"{entity}: Should be returned to pool, but doesn't contain a ReturnToPoolTag component.");
            }

            // Check if entity is mature.
            if (!ageable.ValueRO.ReachedMaturity)
            {
                if (ageable.ValueRO.CurrentAge >= ageable.ValueRO.MaturityAge)
                {
                    ageable.ValueRW.ReachedMaturity = true;
                    reproduce.ValueRW.CanReproduce = true;
                }
            }
        }
    }
}

/// <summary>
/// Handles draining energy for entities.
/// </summary>
[BurstCompile]
public partial struct EnergyDrainSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<EnergyComponent>();
    }

    public void OnUpdate(ref SystemState state)
    {
        foreach (var (energy, entity) in SystemAPI.Query<RefRW<EnergyComponent>>().WithNone<ReturnToPoolTag>().WithEntityAccess())
        {
            // Deduct from energy.
            energy.ValueRW.CurrentEnergy -= SimulationSpeedManager.SimulationSpeed;

            // Check if run out of energy.
            if (energy.ValueRO.CurrentEnergy <= 0)
            {
                if (state.EntityManager.HasComponent<ReturnToPoolTag>(entity))
                {
                    state.EntityManager.SetComponentEnabled<ReturnToPoolTag>(entity, true);
                }
            }
        }
    }
}

/// <summary>
/// Handles locating nearby food sources for entities that are low on energy.
/// </summary>
[BurstCompile]
public partial struct FindFoodSystem : ISystem
{
    private EntityQuery foodQuery;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<EnergyComponent>();
        foodQuery = SystemAPI.QueryBuilder().WithAll<FoodSourceComponent, LocalTransform>().Build();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        // Get count of food sources.
        int foodCount = foodQuery.CalculateEntityCount();
        if (foodCount == 0) return;

        var ecb = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);

        // Create new food data array.
        var foodDataArray = new NativeArray<FoodData>(foodCount, Allocator.TempJob);

        // Populate food data array.
        new PopulateFoodArrayJob
        {
            FoodArray = foodDataArray,
        }.ScheduleParallel(foodQuery, state.Dependency).Complete();

        // Find nearest food source.
        var findFoodJob = new FindNearestFoodJob
        {
            FoodArray = foodDataArray,
            ECB = ecb.AsParallelWriter()
        };
        state.Dependency = findFoodJob.Schedule(state.Dependency);

        // Dispose of native array.
        foodDataArray.Dispose(state.Dependency);
    }
}

/// <summary>
/// Handles moving entities toward located food sources and eating it.
/// </summary>
[BurstCompile]
public partial struct FoodFoundSystem : ISystem
{
    private const float EATING_DISTANCE_SQ = 0.5f * 0.5f;

    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<FoodFoundStateTag>();
    }

    public void OnUpdate(ref SystemState state)
    {
        var ecb = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);

        foreach (var (localTransform, movement, targetFood, energy, entity)
            in SystemAPI.Query<RefRO<LocalTransform>, RefRW<MovementComponent>, RefRO<TargetFoodComponent>, RefRW<EnergyComponent>>()
            .WithAll<FoodFoundStateTag>().WithEntityAccess())
        {
            // Check that food target is still valid.
            if (state.EntityManager.IsEnabled(targetFood.ValueRO.FoodTarget))
            {
                // Get position and distance of target food source.
                float3 foodPosition = state.EntityManager.GetComponentData<LocalTransform>(targetFood.ValueRO.FoodTarget).Position;
                float distanceSq = math.distancesq(localTransform.ValueRO.Position, foodPosition);

                // Set direction towards food source.
                float3 directionToFood = math.normalize(foodPosition - localTransform.ValueRO.Position);
                movement.ValueRW.TargetMovementDirection = directionToFood;

                // Check if near target food source.
                if (distanceSq <= EATING_DISTANCE_SQ)
                {
                    // Eat target food source (mark it to return to object pool).
                    if (state.EntityManager.HasComponent<ReturnToPoolTag>(targetFood.ValueRO.FoodTarget))
                    {
                        state.EntityManager.SetComponentEnabled<ReturnToPoolTag>(targetFood.ValueRO.FoodTarget, true);
                    }

                    // Restore energy.
                    energy.ValueRW.CurrentEnergy = energy.ValueRO.MaxEnergy;

                    // Change state back to wandering.
                    ecb.RemoveComponent<FoodFoundStateTag>(entity);
                    ecb.RemoveComponent<TargetFoodComponent>(entity);
                    ecb.AddComponent<WanderingStateTag>(entity);
                }
            }
            // If target is no longer valid, continue wandering and looking for food.
            else
            {
                ecb.RemoveComponent<FoodFoundStateTag>(entity);
                ecb.RemoveComponent<TargetFoodComponent>(entity);
                ecb.AddComponent<WanderingStateTag>(entity);
            }
        }
    }
}

/// <summary>
/// Handles locating a mate for entities that are ready to reproduce.
/// </summary>
[BurstCompile]
public partial struct FindMateSystem : ISystem
{
    private EntityQuery mateQuery;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<EnergyComponent>();
        mateQuery = SystemAPI.QueryBuilder().WithAll<ReproduceComponent, LocalTransform, AgentTypeComponent>().Build();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        // Get count of potential mates.
        int mateCount = mateQuery.CalculateEntityCount();
        if (mateCount == 0) return;

        var ecb = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);

        // Create new mate data array.
        var mateDataArray = new NativeArray<MateData>(mateCount, Allocator.TempJob);

        // Populate mate data array.
        var populateJobHandle = new PopulateMateArrayJob
        {
            MateArray = mateDataArray
        }.ScheduleParallel(mateQuery, state.Dependency);

        // Find nearest mate.
        var findMateJob = new FindNearestMateJob
        {
            MateArray = mateDataArray,
            ECB = ecb.AsParallelWriter()
        };

        // Schedule findMateJob after populateJobHandle
        state.Dependency = findMateJob.Schedule(populateJobHandle);

        // Dispose of native array.
        mateDataArray.Dispose(state.Dependency);
    }
}

/// <summary>
/// Handles moving entities toward located mates.
/// </summary>
[BurstCompile]
public partial struct FoundMateSystem : ISystem
{
    private const float MATING_DISTANCE_SQ = 0.5f * 0.5f;

    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<MateFoundStateTag>();
    }

    public void OnUpdate(ref SystemState state)
    {
        var ecb = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);

        foreach (var (localTransform, movement, targetMate, reproduce, entity)
            in SystemAPI.Query<RefRO<LocalTransform>, RefRW<MovementComponent>, RefRO<TargetMateComponent>, RefRW<ReproduceComponent>>()
            .WithAll<MateFoundStateTag>().WithEntityAccess())
        {
            // Check that mate target is still valid.
            if (state.EntityManager.IsEnabled(targetMate.ValueRO.MateTarget))
            {
                // Get position and distance of target mate.
                float3 matePosition = state.EntityManager.GetComponentData<LocalTransform>(targetMate.ValueRO.MateTarget).Position;
                float distanceSq = math.distancesq(localTransform.ValueRO.Position, matePosition);

                // Set direction towards mate.
                float3 directionToMate = math.normalize(matePosition - localTransform.ValueRO.Position);
                movement.ValueRW.TargetMovementDirection = directionToMate;

                // Check if near target mate.
                if (distanceSq <= MATING_DISTANCE_SQ)
                {
                    // Attempt to reproduce with target.
                    ecb.RemoveComponent<MateFoundStateTag>(entity);
                    ecb.AddComponent<AttemptingToReproduceTag>(entity);
                }
            }
            // If target is no longer valid, continue wandering.
            else
            {
                ecb.RemoveComponent<MateFoundStateTag>(entity);
                ecb.RemoveComponent<TargetMateComponent>(entity);
                ecb.AddComponent<WanderingStateTag>(entity);
            }
        }
    }
}

/// <summary>
/// Handles attempting to reproduce between two entities.
/// </summary>
[BurstCompile]
public partial struct AttemptingToReproduceSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<AttemptingToReproduceTag>();
    }

    public void OnUpdate(ref SystemState state)
    {
        var ecb = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);

        // Get all entities that are attempting to reproduce.
        var attemptingToReproduce = SystemAPI.QueryBuilder().WithAll<AttemptingToReproduceTag>().Build().ToEntityArray(Allocator.Temp);

        foreach (var (reproduce, targetMate, localTransform, entity)
            in SystemAPI.Query<RefRW<ReproduceComponent>, RefRO<TargetMateComponent>, RefRO<LocalTransform>>()
            .WithAll<AttemptingToReproduceTag>().WithEntityAccess())
        {
            // Check that mate is still valid.
            if (!state.EntityManager.IsEnabled(targetMate.ValueRO.MateTarget))
            {
                ecb.RemoveComponent<AttemptingToReproduceTag>(entity);
                ecb.RemoveComponent<TargetMateComponent>(entity);
                ecb.AddComponent<WanderingStateTag>(entity);
                continue;
            }

            // Check if target mate is also attempting to reproduce.
            if (state.EntityManager.HasComponent<AttemptingToReproduceTag>(targetMate.ValueRO.MateTarget))
            {
                // Check if entity is first to attempt.
                if (entity.Index < targetMate.ValueRO.MateTarget.Index)
                {
                    // Queue entity for reproduce.
                    ecb.AddComponent(entity, new QueueForReproduceTag { SpawnPosition = localTransform.ValueRO.Position });

                    // Clear mate's state.
                    ecb.RemoveComponent<AttemptingToReproduceTag>(targetMate.ValueRO.MateTarget);
                    ecb.RemoveComponent<TargetMateComponent>(targetMate.ValueRO.MateTarget);
                    ecb.AddComponent<WanderingStateTag>(targetMate.ValueRO.MateTarget);
                }

                reproduce.ValueRW.ReadyToReproduce = false;

                ecb.RemoveComponent<AttemptingToReproduceTag>(entity);
                ecb.RemoveComponent<TargetMateComponent>(entity);
                ecb.AddComponent<WanderingStateTag>(entity);
            }
        }

        attemptingToReproduce.Dispose();
    }
}

/// <summary>
/// Handles plants reproducing.
/// </summary>
[BurstCompile]
public partial struct PlantReproduceSystem : ISystem
{
    private Unity.Mathematics.Random random;

    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<ReproduceComponent>();
        this.random = new Unity.Mathematics.Random((uint)state.GlobalSystemVersion);
    }

    public void OnUpdate(ref SystemState state)
    {
        var ecb = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);

        // For each entity that has a reproduce component but not a movementComponent.
        foreach (var (reproduce, localTransform, entity) in SystemAPI.Query<RefRW<ReproduceComponent>, RefRO<LocalTransform>>().WithNone<MovementComponent>().WithEntityAccess())
        {
            if (reproduce.ValueRO.ReadyToReproduce)
            {
                ecb.AddComponent(entity, new QueueForReproduceTag { SpawnPosition = GetRandomSpawnPosition(localTransform.ValueRO.Position) });
                reproduce.ValueRW.ReadyToReproduce = false;
            }
        }
    }

    [BurstCompile]
    private float3 GetRandomSpawnPosition(float3 position)
    {
        const float SPAWN_DISTNACE = 2.5f;

        float randomX = random.NextFloat(-1f, 1f);
        float randomY = random.NextFloat(-1f, 1f);
        float3 randomDirection = new float3(randomX, randomY, 0);
        float3 spawnPosition = position + math.normalize(randomDirection) * SPAWN_DISTNACE;

        return spawnPosition;
    }
}

/// <summary>
/// Handles moving entities.
/// </summary>
[BurstCompile]
public partial struct MovementSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<MovementComponent>();
    }

    public void OnUpdate(ref SystemState state)
    {
        foreach (var (movement, localTransform) in SystemAPI.Query<RefRW<MovementComponent>, RefRW<LocalTransform>>())
        {
            // Update direction and position.
            float lerpFactor = math.clamp(movement.ValueRO.MovementDrag * SimulationSpeedManager.SimulationSpeed, 0f, 1f);
            movement.ValueRW.CurrentMovementDirection = math.lerp(movement.ValueRO.CurrentMovementDirection, movement.ValueRO.TargetMovementDirection, lerpFactor);

            movement.ValueRW.TargetMovementDirection = AvoidBoundariesDirection
                (
                    movement.ValueRO.TargetMovementDirection,
                    localTransform.ValueRO.Position,
                    movement.ValueRO.MinWorldBounds,
                    movement.ValueRO.MaxWorldBounds
                );

            float3 newPosition = localTransform.ValueRO.Position + (movement.ValueRO.CurrentMovementDirection * movement.ValueRO.MovementSpeed * SimulationSpeedManager.SimulationSpeed);

            if (!math.isnan(newPosition.x)) localTransform.ValueRW.Position = newPosition;
        }
    }

    [BurstCompile]
    private float3 AvoidBoundariesDirection(float3 direction, float3 position, float3 minBoundary, float3 maxBoundary)
    {
        float3 newDirection = direction;

        // Check position against boundaries and adjust as needed.
        if (position.x < minBoundary.x)
        {
            newDirection.x = 1;
        }
        else if (position.x > maxBoundary.x)
        {
            newDirection.x = -1;
        }
        if (position.y < minBoundary.y)
        {
            newDirection.y = 1;
        }
        else if (position.y > maxBoundary.y)
        {
            newDirection.y = -1;
        }

        if (math.lengthsq(newDirection) > 0.0001f) return math.normalize(newDirection);
        else return direction;
    }
}

/// <summary>
/// Handles entities wandering sate.
/// </summary>
[BurstCompile]
public partial struct WanderingStateSystem : ISystem
{
    private Unity.Mathematics.Random random;

    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<WanderingStateTag>();
        this.random = new Unity.Mathematics.Random((uint)state.GlobalSystemVersion);
    }

    public void OnUpdate(ref SystemState state)
    {
        foreach (var (wanderingState, movement, localTransform) in SystemAPI.Query<RefRW<WanderingStateTag>, RefRW<MovementComponent>, RefRW<LocalTransform>>())
        {
            // Deduct re-evaluate time from timer.
            movement.ValueRW.ReevaluateTimeRemaining -= SimulationSpeedManager.SimulationSpeed;

            // Check if should re-evaluate.
            if (movement.ValueRO.ReevaluateTimeRemaining <= 0)
            {
                // Reset re-evaluate timer.
                movement.ValueRW.ReevaluateTimeRemaining = movement.ValueRO.ReevaluateInterval;

                // Get change chance.
                float randValue = random.NextFloat();

                // Check if should idle.
                if (movement.ValueRO.IdleChance >= randValue)
                {
                    movement.ValueRW.TargetMovementDirection = float3.zero;
                }
                // Check if should change direction.
                else if (movement.ValueRO.DirectionChangeChance >= randValue)
                {
                    // Change direction.
                    movement.ValueRW.TargetMovementDirection = GetRandomDirection(ref this.random);
                }
            }
        }
    }

    [BurstCompile]
    private float3 GetRandomDirection(ref Unity.Mathematics.Random random)
    {
        float randomX = random.NextFloat(-1f, 1f);
        float randomY = random.NextFloat(-1f, 1f);
        float3 randomDirection = new float3(randomX, randomY, 0f);

        randomDirection = math.normalize(randomDirection);

        return randomDirection;
    }
}

/// <summary>
/// Handles the timer component on entities.
/// </summary>
[BurstCompile]
public partial struct TimerSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<TimerComponent>();
    }

    public void OnUpdate(ref SystemState state)
    {
        foreach (var (timer, entity) in SystemAPI.Query<RefRW<TimerComponent>>().WithEntityAccess())
        {
            if (!timer.ValueRO.IsActive) continue;

            // Deduct time from timer.
            timer.ValueRW.RemainingTime -= SimulationSpeedManager.SimulationSpeed;

            // Check if timer is completed.
            if (timer.ValueRO.RemainingTime <= 0)
            {
                timer.ValueRW.IsActive = false;
            }
        }
    }
}

/// <summary>
/// Handles returning tagged entities to their object pool.
/// </summary>
public partial class ReturnToPoolSystem : SystemBase
{
    private EndSimulationEntityCommandBufferSystem endSimulationEcbSystem;

    protected override void OnCreate()
    {
        endSimulationEcbSystem = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
    }
    protected override void OnUpdate()
    {
        var ecb = endSimulationEcbSystem.CreateCommandBuffer();

        Entities.WithAll<ReturnToPoolTag>().ForEach((Entity entity, in LinkedGameObjectComponent linkedGameObject) =>
        {
            // Check if linked game object is still valid.
            if (linkedGameObject.LinkedGameObject == null)
            {
                ecb.SetComponentEnabled<ReturnToPoolTag>(entity, false);
                return;
            }

            // Check that linked game object is of type IObjectPool and return it to the object pool.
            if (linkedGameObject.LinkedGameObject != null && linkedGameObject.LinkedGameObject.TryGetComponent<IObjectPool>(out IObjectPool agent))
            {
                agent.ReturnToObjectPool();
            }

            // Reset state to Wandering.
            ecb.AddComponent<WanderingStateTag>(entity);

            // Disable entity.
            ecb.SetEnabled(entity, false);
            ecb.SetComponentEnabled<ReturnToPoolTag>(entity, false);
        }).WithoutBurst().Run();

        endSimulationEcbSystem.AddJobHandleForProducer(this.Dependency);
    }
}

/// <summary>
/// Handles spawning new agents from a reproduce queue.
/// </summary>
public partial class QueueReproduceSystem : SystemBase
{
    private EndSimulationEntityCommandBufferSystem endSimulationEcbSystem;

    protected override void OnCreate()
    {
        RequireForUpdate<QueueForReproduceTag>();
        endSimulationEcbSystem = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
    }

    protected override void OnUpdate()
    {
        var ecb = endSimulationEcbSystem.CreateCommandBuffer();

        Entities.ForEach(
            (Entity entity, ref ReproduceComponent reproduce, ref QueueForReproduceTag reproduceQueue,
            in LinkedGameObjectComponent linkedGameObject) =>
        {
            // Check that linked game object is still valid.
            if (linkedGameObject.LinkedGameObject == null)
            {
                ecb.RemoveComponent<QueueForReproduceTag>(entity);
                Debug.Log("Missing game object for entity: " + entity);
                return;
            }

            // Create new agent.
            if (linkedGameObject.LinkedGameObject.TryGetComponent(out EntityAgent agent))
            {
                IObjectPool newPoolObject = ObjectPoolManager.instance?.RequestAgentFromPool(agent.GetType());
                if (newPoolObject != null && newPoolObject is EntityAgent newAgent)
                {
                    newPoolObject.Initialise(reproduceQueue.SpawnPosition, true);
                    ecb.SetEnabled(newAgent.LinkedEntity, true);
                    if (World.DefaultGameObjectInjectionWorld.EntityManager.HasComponent<LocalTransform>(newAgent.LinkedEntity))
                    {
                        World.DefaultGameObjectInjectionWorld.EntityManager.SetComponentData(newAgent.LinkedEntity, new LocalTransform
                        {
                            Scale = 1f,
                            Position = reproduceQueue.SpawnPosition
                        });
                    }
                }
            }
            else Debug.LogWarning($"{entity} could not reproduce as linked game object is not a valid agent type - {agent.GetType().Name}.");

            // Remove queue tag.
            ecb.RemoveComponent<QueueForReproduceTag>(entity);

        }).WithoutBurst().Run();

        endSimulationEcbSystem.AddJobHandleForProducer(this.Dependency);
    }
}

/// <summary>
/// A system that handles selecting an agent and populating the shared data class.
/// </summary>
public partial class AgentDisplayInfoSystem : SystemBase
{
    private const float MAX_RAYCAST_DISTANCE = 1.2f;
    private Entity selectedAgentEntity = Entity.Null;

    protected override void OnCreate()
    {
        RequireForUpdate<LinkedGameObjectComponent>();
    }
    protected override void OnUpdate()
    {
        // Check for a left mouse button click.
        if (Input.GetMouseButtonDown(0))
        {
            // Store the mouse world position.
            float3 mouseWorldPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);

            Entity bestMatchEntity = Entity.Null;
            float closestDistance = float.PositiveInfinity;

            // Go through each entity and check it's position compared to the mouse position. Get the closest one.
            Entities.ForEach((Entity entity, in LinkedGameObjectComponent linkedGameObjectComponent, in LocalTransform transform) =>
            {
                // Check that game object isn't null.
                if (linkedGameObjectComponent.LinkedGameObject == null) return;

                // Calculate the distance from the agent's position to the mouse click.
                float distance = math.distance(transform.Position, mouseWorldPosition);

                // Compare distance.
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    bestMatchEntity = entity;
                }
            }).WithoutBurst().Run();

            // Check if the closest entity is within the max raycast distance.
            if (bestMatchEntity != Entity.Null && closestDistance < MAX_RAYCAST_DISTANCE)
            {
                selectedAgentEntity = bestMatchEntity;
            }
            else
            {
                selectedAgentEntity = Entity.Null;
            }
        }

        // Update information if there is an agent selected.
        if (selectedAgentEntity != Entity.Null && EntityManager.Exists(selectedAgentEntity))
        {
            // Get all relevant component data.
            var agentType = EntityManager.GetComponentData<AgentTypeComponent>(selectedAgentEntity);
            var ageable = EntityManager.GetComponentData<AgeableComponent>(selectedAgentEntity);
            var linkedGameObject = EntityManager.GetComponentObject<LinkedGameObjectComponent>(selectedAgentEntity);
            var reproduce = EntityManager.GetComponentData<ReproduceComponent>(selectedAgentEntity);

            // Check optional data.
            bool hasEnergy = EntityManager.HasComponent<EnergyComponent>(selectedAgentEntity);
            var energy = hasEnergy ? EntityManager.GetComponentData<EnergyComponent>(selectedAgentEntity) : new EnergyComponent();

            // Populate the singleton data class.
            AgentInfoData.instance.IsAgentSelected = true;
            AgentInfoData.instance.AgentName = linkedGameObject.LinkedGameObject.name.Replace("(Clone)", "");
            AgentInfoData.instance.CurrentAge = ageable.CurrentAge;
            AgentInfoData.instance.LifeExpectancy = ageable.LifeExpectancy;
            AgentInfoData.instance.AgentGender = reproduce.Gender.ToString();
            AgentInfoData.instance.CurrentEnergy = hasEnergy ? energy.CurrentEnergy : 0f;
            AgentInfoData.instance.MaxEnergy = hasEnergy ? energy.MaxEnergy : 0f;
            AgentInfoData.instance.LinkedGameObject = linkedGameObject.LinkedGameObject;

            // Determine the state from the tag components.
            if (EntityManager.HasComponent<WanderingStateTag>(selectedAgentEntity))
            {
                AgentInfoData.instance.AgentState = "Wandering";
            }
            else if (EntityManager.HasComponent<FoodFoundStateTag>(selectedAgentEntity))
            {
                AgentInfoData.instance.AgentState = "Searching for Food";
            }
            else if (EntityManager.HasComponent<MateFoundStateTag>(selectedAgentEntity))
            {
                AgentInfoData.instance.AgentState = "Searching for Mate";
            }
            else if (EntityManager.HasComponent<AttemptingToReproduceTag>(selectedAgentEntity))
            {
                AgentInfoData.instance.AgentState = "Reproducing";
            }
            else
            {
                AgentInfoData.instance.AgentState = "Idle/Other";
            }
        }
        // If nothing is selected, clear the data.
        else
        {
            AgentInfoData.instance.IsAgentSelected = false;
        }
    }
}


# endregion

#region Components

/// <summary>
/// Stores what type of agent this entity is.
/// </summary>
public partial struct AgentTypeComponent : IComponentData
{
    public enum AgentType
    {
        Predator,
        Prey,
        Plant
    }
    public AgentType TypeOfAgent;
}

/// <summary>
/// Stores movement data.
/// </summary>
public partial struct MovementComponent : IComponentData
{
    public float MovementSpeed;
    public float3 TargetMovementDirection;
    public float3 CurrentMovementDirection;
    public float MovementDrag;
    public float3 MinWorldBounds;
    public float3 MaxWorldBounds;

    public float ReevaluateInterval;
    public float DirectionChangeChance;
    public float IdleChance;
    public float ReevaluateTimeRemaining;
}

/// <summary>
/// Stores life expectancy data.
/// </summary>
public partial struct AgeableComponent : IComponentData
{
    public float CurrentAge;
    public float LifeExpectancy;
    public float MaturityAge;
    public bool ReachedMaturity;
}

/// <summary>
/// Stores energy data.
/// </summary>
public partial struct EnergyComponent : IComponentData
{
    public float CurrentEnergy;
    public float MaxEnergy;
    public float HighThreshold;
    public float LowThreshold;
}

/// <summary>
/// Stores type of <see cref="FoodSourceComponent.FoodSourceType"/> this entity eats.
/// </summary>
public partial struct HungerComponent : IComponentData
{
    public FoodSourceComponent.FoodSourceType PreferredFoodSource;
}

/// <summary>
/// Stores the type of <see cref="FoodSourceComponent.FoodSourceType"/> this entity is considered.
/// </summary>
public partial struct FoodSourceComponent : IComponentData
{
    public enum FoodSourceType
    {
        Prey,
        Plant
    }
    public FoodSourceType TypeOfFoodSource;
}

/// <summary>
/// Stores reproduction data.
/// </summary>
public partial struct ReproduceComponent : IComponentData
{
    public enum Genders
    {
        Undefined,
        Male,
        Female
    }

    public float ReproduceCooldown;
    public bool CanReproduce;
    public bool ReadyToReproduce;
    public Genders Gender;
}

/// <summary>
/// Stores timer data.
/// </summary>
public partial struct TimerComponent : IComponentData
{
    public enum TimerType
    {
        Reproduce,
    }
    public TimerType TypeOfTimer;
    public float RemainingTime;
    public bool IsActive;
}

/// <summary>
/// Stores a target <see cref="FoodSourceComponent.FoodSourceType"/> entity.
/// </summary>
public partial struct TargetFoodComponent : IComponentData
{
    public Entity FoodTarget;
}

/// <summary>
/// Stores a target entity for reproduction.
/// </summary>
public partial struct TargetMateComponent : IComponentData
{
    public Entity MateTarget;
}

/// <summary>
/// Tag for retuning the entity to the object pool.
/// </summary>
public partial struct ReturnToPoolTag : IComponentData, IEnableableComponent { }

/// <summary>
/// Wandering state tag.
/// </summary>
public partial struct WanderingStateTag : IComponentData { }
/// <summary>
/// Food found state tag.
/// </summary>
public partial struct FoodFoundStateTag : IComponentData { }

/// <summary>
/// Mate found state tag.
/// </summary>
public partial struct MateFoundStateTag : IComponentData { }

/// <summary>
/// Tag for entity that is attempting to reproduce.
/// </summary>
public partial struct AttemptingToReproduceTag : IComponentData { }

/// <summary>
/// Tag for entity that is queued to reproduce.
/// </summary>
public partial struct QueueForReproduceTag : IComponentData
{
    public float3 SpawnPosition;
}

/// <summary>
/// Stores a reference to a linked <see cref="GameObject"/>.
/// </summary>
public partial class LinkedGameObjectComponent : IComponentData
{
    public GameObject LinkedGameObject;
}

#endregion

# region Data

/// <summary>
/// Stores data for <see cref="FindNearestFoodJob"/>.
/// </summary>
public struct FoodData
{
    public Entity Entity;
    public float3 Position;
    public FoodSourceComponent.FoodSourceType TypeOfFoodSource;
}

/// <summary>
/// Stores data for <see cref="FindNearestMateJob"/>.
/// </summary>
public struct MateData
{
    public Entity Entity;
    public float3 Position;
    public ReproduceComponent.Genders Gender;
    public bool CanReproduce;
    public AgentTypeComponent.AgentType AgentType;
}

# endregion

#region Settings

/// <summary>
/// Stores common settings for all agents.
/// </summary>
[Serializable]
public abstract class AgentSettings
{
    public SpawnSettings SpawnSettings;
    public LifeExpectancySettings LifeExpectancySettings;
    public ReproductionSettings ReproductionSettings;
}

/// <summary>
/// Stores settings for predator agents.
/// </summary>
[Serializable]
public class PredatorSettings : AgentSettings
{
    public MovementSettings MovementSettings;
    public EnergySettings EnergySettings;
    public HungerSettings HungerSettings;
}

/// <summary>
/// Stores settings for prey agents.
/// </summary>
[Serializable]
public class PreySettings : AgentSettings
{
    public MovementSettings MovementSettings;
    public EnergySettings EnergySettings;
    public HungerSettings HungerSettings;
    public FoodSourceSettings FoodSourceSettings;
}

/// <summary>
/// Stores settings for plant agents.
/// </summary>
[Serializable]
public class PlantSettings : AgentSettings
{
    public FoodSourceSettings FoodSourceSettings;
}

/// <summary>
/// Settings for spawning objects within an object pool.
/// </summary>
[Serializable]
public struct SpawnSettings
{
    [Tooltip("The prefab to spawn.")]
    public GameObject EntityPrefab;
    [Tooltip("The amount of objects that will be stored in an object pool.")]
    public int ObjectPoolAmount;
    [Tooltip("The initial amount to spawn into the simulation.")]
    public int InitialSpawnAmount;
    [Tooltip("The parent container for this type of entity.")]
    public Transform EntityParentContainer;
}

/// <summary>
/// Settings for entities with a life expectancy.
/// </summary>
[Serializable]
public struct LifeExpectancySettings
{
    [Tooltip("How long is the entity expected to live for in MINUTES.")]
    public float LifeExpectancy;
    [Tooltip("LifeExpectancy will vary by this value.")]
    public float LifeExpectancyVariance;
    [Tooltip("At what age will the entity reach maturity in MINUTES.")]
    public float MaturityAge;
    [Tooltip("The range the entity will be scaled from 0 to MaturityAge.")]
    public Vector2 AgeScaleRange;
}

/// <summary>
/// Settings for entities with movement.
/// </summary>
[Serializable]
public struct MovementSettings
{
    [Tooltip("The speed the entity moves at.")]
    public float MovementSpeed;
    [Tooltip("MovementSpeed will vary by this value for each entity.")]
    public float SpeedVariance;
    [Tooltip("The internal between checking for a direction change in Seconds.")]
    public float ReevaluateInterval;
    [Tooltip("The chance (between 0.0 and 1.0) that the entity will change direction."), Range(0f, 1f)]
    public float DirectionChangeChance;
    [Tooltip("The chance (between 0.0 and 1.0) that the entity will idle instead of changing direction."), Range(0f, 1f)]
    public float IdleChance;
    [Tooltip("The drag applied to movement.")]
    public float MovementDrag;
}

/// <summary>
/// Settings for entities with an energy level.
/// </summary>
[Serializable]
public struct EnergySettings
{
    [Tooltip("The max energy level of an entity in SECONDS.")]
    public float EnergyMax;
    [Tooltip("EnergyMax will vary by this value.")]
    public float EnergyVariance;
    [Tooltip("The threshold for what is considered a low energy state.")]
    public float LowEnergyThreshold;
    [Tooltip("The threshold for what is considered a high energy state.")]
    public float HighEnergyThreshold;
}

/// <summary>
/// Settings for entities that eat.
/// </summary>
[Serializable]
public struct HungerSettings
{
    [Tooltip("The type of food an entity prefers to eat.")]
    public FoodSourceComponent.FoodSourceType PreferredFoodType;
}

/// <summary>
/// Settings for entities that are a food source.
/// </summary>
[Serializable]
public struct FoodSourceSettings
{
    [Tooltip("The type of food source this entity is.")]
    public FoodSourceComponent.FoodSourceType TypeOfFoodSource;
}

/// <summary>
/// Settings for entities that can reproduce.
/// </summary>
[Serializable]
public struct ReproductionSettings
{
    [Tooltip("The cooldown duration in SECONDS before the entity can reproduce again.")]
    public float ReproduceCooldown;
}

/// <summary>
/// Settings for the camera.
/// </summary>
[Serializable]
public struct CameraSettings
{
    [Tooltip("The zoom range of the camera.")]
    public Vector2 ZoomRange;
    [Tooltip("The zoom sensitivity of the camera.")]
    public float ZoomSensitivity;
}

#endregion

#region UI and User Functonality

/// <summary>
/// Handles the camera movement and zoom.
/// </summary>
public class CameraMovement : MonoBehaviour
{
    // --- VARIABLES ---

    // Public References and Values
    public CameraSettings CameraData;

    // Internal References and Values
    private CinemachineCamera virtualCamera;
    private Camera mainCamera;
    private bool isPanning = false;
    private Vector3 originPositionWorld;


    // --- METHODS ---
    private void Start()
    {
        virtualCamera = FindFirstObjectByType<CinemachineCamera>();
        if (virtualCamera == null) Debug.LogError($"Cannot locate the Cinemachine Camera in the scene.");
        mainCamera = Camera.main;
    }

    private void LateUpdate()
    {
        if (virtualCamera == null) return;

        // --- Zoom ---
        Vector2 mouseScroll = Input.mouseScrollDelta;
        if (mouseScroll.y != 0)
        {
            virtualCamera.Lens.OrthographicSize = Mathf.Clamp(
                virtualCamera.Lens.OrthographicSize += (-mouseScroll.y * CameraData.ZoomSensitivity),
                CameraData.ZoomRange.x,
                CameraData.ZoomRange.y
                );
        }

        // --- Move ---
        if (Input.GetMouseButtonDown(2) && !isPanning)
        {
            originPositionWorld = GetMouseWorldPosition();
            isPanning = true;
        }

        if (Input.GetMouseButtonUp(2))
        {
            isPanning = false;
        }

        if (Input.GetAxisRaw("Horizontal") != 0 || Input.GetAxisRaw("Vertical") != 0)
        {
            // Get movement direction.
            Vector3 movementDirection = new Vector3(Input.GetAxisRaw("Horizontal"), Input.GetAxisRaw("Vertical"), 0);
            movementDirection.Normalize();
            Vector3 targetPosition = virtualCamera.transform.position + movementDirection;

            // Update camera position
            virtualCamera.transform.position = targetPosition;
            ClampCameraPosition();
        }

        if (isPanning)
        {
            // Get movement direction.
            Vector3 direction = originPositionWorld - GetMouseWorldPosition();
            Vector3 targetPosition = virtualCamera.transform.position + direction;

            // Update camera position
            virtualCamera.transform.position = targetPosition;
            ClampCameraPosition();
        }
    }

    private Vector3 GetMouseWorldPosition()
    {
        return mainCamera.ScreenToWorldPoint(Input.mousePosition);
    }

    private void ClampCameraPosition()
    {
        Bounds worldBounds = SimulationManager.instance.WorldBounds;
        float xClamped = Mathf.Clamp(virtualCamera.transform.position.x, worldBounds.min.x, worldBounds.max.x);
        float yClamped = Mathf.Clamp(virtualCamera.transform.position.y, worldBounds.min.y, worldBounds.max.y);

        virtualCamera.transform.position = new Vector3(xClamped, yClamped, -1);
    }
}

/// <summary>
/// Handles the simulation speed and user input to change it.
/// </summary>
public class SimulationSpeedManager : MonoBehaviour
{
    // --- VARIABLES ---
    private const int MIN_SIMULATION_SPEED = 1;
    private const int MAX_SIMULATION_SPEED = 1000;

    /// <summary>
    /// The speed of the simulation, taking <see cref="Time.deltaTime"/> into account.
    /// </summary>
    public static float SimulationSpeed { get; private set; }
    private Slider simulationSpeedSlider;
    private TMP_Text simulationSpeedDisplayText;
    private TMP_InputField simulationSpeedInputField;
    [Tooltip("The simulation speed multiplier. ")]
    [SerializeField, Range(MIN_SIMULATION_SPEED, MAX_SIMULATION_SPEED)] private int simulationSpeedMultiplier;


    // --- METHODS ---
    /// <summary>
    /// To be when this class is created.
    /// </summary>
    public void Initialise(Slider simulationSpeedSlider, TMP_Text simulationSpeedDisplayText, TMP_InputField simulationSpeedInputField)
    {
        // Set default simulation speed.
        UpdateSimulationSpeed(MIN_SIMULATION_SPEED);

        // Assign references.
        this.simulationSpeedSlider = simulationSpeedSlider;
        this.simulationSpeedDisplayText = simulationSpeedDisplayText;
        this.simulationSpeedInputField = simulationSpeedInputField;

        // Set up simulation speed slider values.
        this.simulationSpeedSlider.minValue = MIN_SIMULATION_SPEED;
        this.simulationSpeedSlider.maxValue = MAX_SIMULATION_SPEED;
        this.simulationSpeedSlider.value = SimulationSpeed;

        // Connect to events.
        this.simulationSpeedInputField?.onSubmit.AddListener(OnSimulationSpeedInputSubmit);
        this.simulationSpeedSlider?.onValueChanged.AddListener(OnSimulationValueSliderChanged);
    }

    private void OnDisable()
    {
        // Disconnect from events.
        simulationSpeedSlider?.onValueChanged.RemoveListener(OnSimulationValueSliderChanged);
        simulationSpeedInputField?.onSubmit.RemoveListener(OnSimulationSpeedInputSubmit);
    }

    private void Update()
    {
        // Set simulation speed.
        SimulationSpeed = simulationSpeedMultiplier * Time.deltaTime;
    }

    private void UpdateSimulationSpeed(int value)
    {
        // Update speed multiplier.
        simulationSpeedMultiplier = Mathf.Clamp(value, MIN_SIMULATION_SPEED, MAX_SIMULATION_SPEED);

        // Update UI.
        if (simulationSpeedSlider == null || simulationSpeedDisplayText == null) return;
        simulationSpeedSlider.value = value;
        simulationSpeedDisplayText.text = $"{value}x Speed";
    }

    private void OnSimulationValueSliderChanged(float value)
    {
        UpdateSimulationSpeed((int)value);
    }

    private void OnSimulationSpeedInputSubmit(string value)
    {
        if (int.TryParse(value, out int result))
        {
            UpdateSimulationSpeed(result);
        }
    }
}

/// <summary>
/// A singleton class to hold agent data for the UI.
/// </summary>
public class AgentInfoData
{
    public static AgentInfoData instance { get; private set; } = new AgentInfoData();
    public bool IsAgentSelected = false;

    // Agent data properties to be populated by the ECS system
    public string AgentName;
    public string AgentState;
    public string AgentGender;
    public float CurrentAge;
    public float LifeExpectancy;
    public float CurrentEnergy;
    public float MaxEnergy;

    // Reference to the GameObject for highlighting and sprite
    public GameObject LinkedGameObject;

    private AgentInfoData() { }
}

/// <summary>
/// Handles displaying agent information based on user input.
/// </summary>
public class AgentInfoDisplayManager : MonoBehaviour
{
    // --- VARIABLES ---
    private const float RAYCAST_DISTANCE = 5f;

    private TMP_Text agentTitleText;
    private TMP_Text agentStateText;
    private TMP_Text agentEnergyText;
    private TMP_Text agentAgeText;
    private TMP_Text agentGenderText;
    private Image agentImage;
    private GameObject infoPanelTransform;
    private Transform agentHighlightTransform;
    private GameObject currentTargetAgent;
    private SpriteRenderer targetSpriteRenderer;
    private AgentInfoData infoData;


    // --- METHODS ---
    /// <summary>
    /// Should be called when this class is instantiated.
    /// </summary>
    public void Initialise(
        TMP_Text agentTitleText,
        TMP_Text agentStateText,
        TMP_Text agentEnergyText,
        TMP_Text agentAgeText,
        TMP_Text agentGenderText,
        Image agentImage,
        GameObject infoPanelTransform,
        Transform agentHighlightTransform
        )
    {
        this.agentTitleText = agentTitleText;
        this.agentStateText = agentStateText;
        this.agentEnergyText = agentEnergyText;
        this.agentAgeText = agentAgeText;
        this.agentGenderText = agentGenderText;
        this.agentImage = agentImage;
        this.infoPanelTransform = infoPanelTransform;
        this.agentHighlightTransform = agentHighlightTransform;
    }

    private void Start()
    {
        infoData = AgentInfoData.instance;

        TurnOffDisplay();
    }

    private void LateUpdate()
    {
        if (infoData.IsAgentSelected)
        {
            TurnOnDisplay();
            UpdateDisplay();
        }
        else TurnOffDisplay();
    }

    private void UpdateDisplay()
    {
        // Check if agent is still valid
        if (!infoData.LinkedGameObject.activeInHierarchy)
        {
            TurnOffDisplay();
            return;
        }
        
        // Update live information.
        agentAgeText.text = "Age: " + infoData.CurrentAge.ToString("0.00") + " / " + infoData.LifeExpectancy.ToString("0.00") + " mins";
        agentStateText.text = "State: " + infoData.AgentState;
        agentGenderText.text = "Gender: " + infoData.AgentGender;

        // Only update energy text if a predator or prey is selected.
        if (agentEnergyText.gameObject.activeSelf)
        {
            agentEnergyText.text = "Energy: " + infoData.CurrentEnergy.ToString("0.00") + " / " + infoData.MaxEnergy.ToString("0.00");
        }
    }
    private void TurnOnDisplay()
    {
        if (infoPanelTransform.activeSelf && currentTargetAgent == infoData.LinkedGameObject && infoData.LinkedGameObject.activeInHierarchy) return;

        infoPanelTransform.SetActive(true);
        currentTargetAgent = infoData.LinkedGameObject;

        // Get the SpriteRenderer from the linked GameObject and update the profile image.
        if (infoData.LinkedGameObject != null && infoData.LinkedGameObject.TryGetComponent(out targetSpriteRenderer))
        {
            agentImage.gameObject.SetActive(true);
            agentImage.sprite = targetSpriteRenderer.sprite;
        }
        else
        {
            agentImage.gameObject.SetActive(false);
        }

        // Set agent highlight parent to selected agent.
        if (agentHighlightTransform.parent != infoData.LinkedGameObject.transform && targetSpriteRenderer != null)
        {
            // Set highlight to agent's position with sprite offset and parent it to target.
            agentHighlightTransform.gameObject.SetActive(true);
            agentHighlightTransform.position = infoData.LinkedGameObject.transform.position + new Vector3(0, (targetSpriteRenderer.bounds.size.y / 2), 0);
            agentHighlightTransform.SetParent(infoData.LinkedGameObject.transform);
        }
        else
        {
            agentHighlightTransform.gameObject.SetActive(false);
        }

        // Update the static information once.
        agentTitleText.text = infoData.AgentName;

        // Activate optional info.
        if (infoData.MaxEnergy > 0)
        {
            agentEnergyText.gameObject.SetActive(true);
        }
        else
        {
            agentEnergyText.gameObject.SetActive(false);
        }
    }

    private void TurnOffDisplay()
    {
        if (!infoPanelTransform.activeSelf) return;

        infoPanelTransform.SetActive(false);
        agentHighlightTransform.gameObject.SetActive(false);
    }
}

#endregion