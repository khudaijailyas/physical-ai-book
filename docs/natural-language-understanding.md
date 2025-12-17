# Natural Language Understanding

## Learning Objectives

- Understand how robots extract meaning and intent from text produced by speech recognition
- Learn the key components of natural language understanding including tokenization, parsing, and semantic analysis
- Identify how robots handle ambiguity, context, and different ways of expressing the same idea
- Recognize the difference between understanding literal words and understanding what people actually mean
- Understand practical approaches for implementing natural language understanding in robotic systems

## Concept Explanation

### What is Natural Language Understanding?

Natural language understanding, often called NLU, is the process by which robots figure out what text actually means. After speech recognition converts spoken words into written text, the robot faces a new challenge: understanding what those words mean and what the person wants.

Consider the sentence "Can you open the door?" The words are simple, but understanding requires more than reading them. The robot must recognize this is a polite request, not a question about its capabilities. It must identify "open" as an action and "door" as the object to act upon. It must determine which door if multiple doors exist nearby.

Natural language understanding bridges the gap between words on a page and actionable meaning that robots can use to respond appropriately.

### Why Understanding Language is Difficult

Human language is remarkably complex. The same idea can be expressed in countless ways. "Open the door," "Please open the door," "Could you get the door?" and "The door needs opening" all request the same action but use completely different words and structures.

Language is also full of ambiguity. The word "bank" might mean a financial institution or the side of a river. The sentence "I saw the man with the telescope" could mean you used a telescope to see him, or you saw a man who was holding a telescope.

Humans handle this complexity effortlessly because we understand context, have world knowledge, and can infer unstated information. Robots must learn to do the same through natural language understanding systems.

### The Components of Natural Language Understanding

Natural language understanding involves several interconnected processes. Let us explore each one.

**Tokenization** is the first step. It breaks text into individual units called tokens. Usually, tokens are words, but they can also include punctuation marks or parts of words.

The sentence "Hello, how are you?" becomes six tokens: "Hello", ",", "how", "are", "you", and "?". This segmentation allows the system to analyze each piece separately.

Tokenization seems simple but has challenges. Should "don't" be one token or two ("do" and "n't")? What about "New York" or "ice cream"? Different tokenization strategies affect how the system processes language.

**Part-of-Speech Tagging** identifies what grammatical role each word plays. Is a word a noun, verb, adjective, or something else? This helps the system understand sentence structure.

In "The robot moves quickly," the tagger identifies "robot" as a noun, "moves" as a verb, and "quickly" as an adverb. Understanding these roles helps parse the sentence correctly.

**Named Entity Recognition** identifies specific entities mentioned in text. These include people's names, places, organizations, dates, times, and other important references.

In "Meet me at Central Station on Tuesday," the system recognizes "Central Station" as a location and "Tuesday" as a time reference. This allows the robot to extract structured information from natural text.

**Dependency Parsing** analyzes the grammatical structure of sentences. It determines which words modify which other words and how different parts of the sentence relate to each other.

For "The red ball rolls quickly," parsing reveals that "red" modifies "ball," "ball" is the subject of "rolls," and "quickly" modifies "rolls." This structure helps extract meaning.

**Semantic Analysis** interprets the actual meaning of words and sentences. This goes beyond grammar to understand what the text is trying to convey.

Semantic analysis recognizes that "buy," "purchase," and "acquire" have similar meanings. It understands that "The temperature is dropping" means it is getting colder, not that someone is physically dropping something.

**Intent Recognition** determines what the speaker wants to accomplish. Is this a question, command, statement, or request? What action, if any, should the robot take?

The utterance "It's cold in here" might be a simple observation, but more likely it is an indirect request to increase the temperature. Intent recognition helps the robot understand the underlying purpose.

**Slot Filling** extracts specific pieces of information needed to fulfill a request. Once the robot knows the intent, it must identify relevant details.

For "Set an alarm for 7 AM tomorrow," the intent is setting an alarm. The slots are time (7 AM) and date (tomorrow). The robot extracts these values to complete the action.

### How Natural Language Understanding Works Step by Step

Let us walk through how a robot processes a sentence using natural language understanding.

**Step 1: Receiving the Text**

The robot receives text from the speech recognition system. For example: "Please show me the weather forecast for Paris tomorrow."

**Step 2: Preprocessing**

The system cleans and normalizes the text. It might convert everything to lowercase, expand contractions like "I'm" to "I am," or correct common spelling errors.

**Step 3: Tokenization**

The sentence is split into tokens: ["Please", "show", "me", "the", "weather", "forecast", "for", "Paris", "tomorrow", "."]

**Step 4: Part-of-Speech Tagging**

Each token receives a grammatical label:
- Please: Interjection
- show: Verb
- me: Pronoun
- the: Determiner
- weather: Noun
- forecast: Noun
- for: Preposition
- Paris: Proper noun
- tomorrow: Noun
- .: Punctuation

**Step 5: Named Entity Recognition**

The system identifies:
- "Paris" as a location (city)
- "tomorrow" as a time reference

**Step 6: Dependency Parsing**

The parser determines that "show" is the main verb, "me" is the indirect object, "forecast" is the direct object, "weather" modifies "forecast," and "for Paris tomorrow" provides additional context about when and where.

**Step 7: Semantic Analysis**

The system interprets the meaning:
- "show" means to display or present information
- "weather forecast" is a prediction of future weather conditions
- "Paris" specifies the geographic location
- "tomorrow" specifies the time period

**Step 8: Intent Recognition**

The system classifies the overall intent as "get_weather_forecast." This is a request for information retrieval, not a command to perform a physical action.

**Step 9: Slot Filling**

The system extracts structured information:
- Intent: get_weather_forecast
- Location: Paris
- Time: tomorrow
- Action: show/display

**Step 10: Output Generation**

The understanding system produces a structured representation that other robot systems can use:

```
{
  "intent": "get_weather_forecast",
  "slots": {
    "location": "Paris",
    "time": "tomorrow"
  },
  "action": "display_information"
}
```

This structured output goes to the robot's dialogue management system, which decides how to respond.

### Handling Ambiguity

Language is full of ambiguous statements that could have multiple meanings. Natural language understanding systems must resolve these ambiguities.

**Lexical Ambiguity** occurs when words have multiple meanings. The word "play" could mean engaging in a game, performing music, or watching a theatrical production.

Systems use context to disambiguate. In "play the guitar," the musical meaning is clear. In "watch a play," the theatrical meaning fits. Analyzing surrounding words helps select the correct interpretation.

**Syntactic Ambiguity** happens when sentence structure allows multiple interpretations. "I saw the person with binoculars" could mean you used binoculars to see them, or the person you saw was holding binoculars.

Advanced parsing techniques and world knowledge help resolve these cases. The system might consider which interpretation is more probable given the context.

**Referential Ambiguity** occurs with pronouns and references. In "The robot brought the box. It was heavy," does "it" refer to the robot or the box?

Coreference resolution systems track entities mentioned in conversation and determine what pronouns refer to. This requires understanding both grammar rules and logical reasoning.

### Understanding Context

Meaning depends heavily on context. The same words mean different things in different situations.

**Dialogue Context** refers to what was said earlier in the conversation. If someone asks "What about red?" the robot must remember what object was being discussed previously.

Systems maintain conversation history and use it to interpret current utterances. This allows natural back-and-forth exchanges.

**Situational Context** includes the physical environment and current circumstances. "Bring it here" means different things depending on what "it" is and where "here" is.

Robots must integrate information from their sensors and knowledge of their environment with language understanding.

**Cultural and Social Context** affects meaning too. Politeness conventions, idioms, and indirect speech vary across cultures. "Would you mind closing the door?" is a polite request in English-speaking cultures, not actually asking about mental state.

### Different Approaches to Natural Language Understanding

Several methods exist for implementing natural language understanding.

**Rule-Based Systems** use manually written rules to parse and interpret language. Linguists and engineers define grammar rules and semantic patterns.

These systems work well for limited domains with predictable language. They are transparent and controllable but require extensive manual effort and struggle with language variation.

**Statistical Methods** learn patterns from large amounts of example text. They use probability models to predict the most likely interpretation.

These methods handle variation better than rules but require substantial training data. They were the dominant approach for many years.

**Machine Learning with Feature Engineering** trains models on labeled examples but requires humans to define relevant features. Engineers specify which characteristics of text the model should consider.

This approach combines human expertise with automatic learning. It works well for specific tasks like intent classification.

**Deep Learning Methods** use neural networks to automatically learn relevant features and patterns from data. These models discover important characteristics without manual feature engineering.

Deep learning has become the standard approach because it achieves better accuracy across diverse language understanding tasks. Models learn from millions of example sentences.

**Large Language Models** are massive neural networks trained on enormous amounts of text from the internet. They learn broad language understanding capabilities applicable to many tasks.

Models like BERT, GPT, and their successors can be adapted to specific robot tasks with relatively little additional training. They understand context, nuance, and can perform various language understanding operations.

**Hybrid Systems** combine multiple approaches. They might use rule-based systems for structured commands, statistical methods for intent classification, and large language models for open-ended understanding.

This flexibility allows optimizing for different situations and requirements.

## Why This Matters

### Enabling Meaningful Robot Interaction

Without natural language understanding, robots can only recognize words without grasping their meaning. Understanding transforms robots from simple speech-to-text machines into intelligent agents that comprehend what people want and why.

This capability is fundamental for any robot that must interact with humans naturally. The robot must go beyond hearing words to understanding intentions, preferences, and needs.

### Supporting Diverse Communication Styles

People express ideas differently. Some speak formally, others casually. Some are direct, others hint at what they want. Natural language understanding allows robots to handle this diversity.

A service robot must understand both "Excuse me, could you possibly direct me to the restroom?" and "Where's the bathroom?" even though these express the same need using completely different language.

### Critical for Humanoid Robots

Humanoid robots work alongside humans as assistants, companions, and colleagues. These robots must understand nuanced communication including indirect requests, politeness conventions, and contextual references.

A humanoid nurse assistant might hear "The patient in room 3 seems uncomfortable." This is not just stating a fact. It is an implicit request to check on the patient. Understanding this requires sophisticated natural language processing.

### Enabling Complex Task Execution

Many robot tasks require understanding detailed instructions. "Go to the kitchen, find the red box on the counter, and bring it to the living room" involves multiple actions, object specifications, and location references.

Natural language understanding breaks this down into structured components the robot can execute: navigate to kitchen, locate object matching description (red box on counter), grasp object, navigate to living room, release object.

### Improving Service Robot Effectiveness

Service robots in hotels, hospitals, and retail environments answer questions and fulfill requests from diverse users. These robots must understand questions phrased in countless ways.

"What time does the restaurant open?" "When can I get breakfast?" and "Is the dining room available now?" all seek similar information but require understanding the underlying intent.

### Handling Multi-Turn Conversations

Real conversations involve multiple exchanges building on each other. Understanding requires tracking what was discussed and how new utterances relate to previous ones.

```
User: "What movies are playing tonight?"
Robot: "There are three movies: Action Hero, Romantic Comedy, and Space Adventure."
User: "What about the second one?"
```

The robot must understand "the second one" refers to "Romantic Comedy" from its previous response. This contextual understanding is essential for natural dialogue.

### Safety-Critical Communication

In some applications, misunderstanding language can be dangerous. Emergency response robots, surgical assistants, and industrial robots must accurately understand critical commands.

Natural language understanding with high confidence detection helps ensure robots correctly interpret important instructions and ask for clarification when uncertain.

## Example

### A Kitchen Helper Robot Understanding a Request

Let us examine how a kitchen assistant robot processes a cooking-related request.

**User:** "Can you help me make pasta? I want something with tomatoes and cheese."

**Robot:** "I can help you make pasta with tomatoes and cheese. Would you like me to suggest a recipe for pasta with marinara sauce and mozzarella?"

### How the System Processes This Request

**Receiving Input:** The speech recognition system sends the text to natural language understanding: "Can you help me make pasta? I want something with tomatoes and cheese."

**Tokenization:** The system breaks this into tokens. The first sentence becomes: ["Can", "you", "help", "me", "make", "pasta", "?"]. The second sentence becomes: ["I", "want", "something", "with", "tomatoes", "and", "cheese", "."]

**Part-of-Speech Tagging:** The tagger identifies grammatical roles:
- First sentence: "Can" (modal verb), "you" (pronoun), "help" (verb), "me" (pronoun), "make" (verb), "pasta" (noun)
- Second sentence: "I" (pronoun), "want" (verb), "something" (pronoun), "with" (preposition), "tomatoes" (noun), "and" (conjunction), "cheese" (noun)

**Named Entity Recognition:** The system identifies food-related entities:
- "pasta" - food item, main dish
- "tomatoes" - ingredient, vegetable
- "cheese" - ingredient, dairy product

**Dependency Parsing:** The parser analyzes sentence structure. In the first sentence, "help" is the main verb, "you" is the subject, "me" is the object, and "make pasta" is the purpose clause. In the second sentence, "want" is the main verb, "I" is the subject, and "something with tomatoes and cheese" is the object.

**Semantic Analysis:** The system interprets meaning at a deeper level:
- "Can you help me" is a polite request form, not literally asking about capability
- "make pasta" means prepare/cook pasta, not manufacture it
- "I want something with" expresses preference or requirement for ingredients
- The two sentences are related: the second provides specifications for the first

**Intent Recognition:** The system determines the primary intent is "request_cooking_assistance." This is not a simple question requiring information, but a request for the robot to help with a task.

The system also recognizes a secondary intent of "specify_preferences" in the second sentence, providing constraints on the desired outcome.

**Slot Filling:** The system extracts structured information:
- Primary intent: request_cooking_assistance
- Task type: cooking
- Dish category: pasta
- Required ingredients: tomatoes, cheese
- User role: requester seeking help
- Robot role: assistant providing guidance or action

**Contextual Understanding:** The system recognizes several important contextual elements:

The request is polite and collaborative ("can you help") rather than commanding, suggesting the user expects an interactive process. The two sentences form a single unified request, with the second sentence clarifying requirements for the first. The user wants to participate ("help me make") rather than have the robot do everything.

**Constraint Processing:** The system identifies "tomatoes" and "cheese" as required ingredients, meaning any solution must include these items. The robot must suggest recipes or procedures that satisfy these constraints.

**Knowledge Integration:** The natural language understanding system consults the robot's knowledge base about cooking. It knows:
- Pasta dishes commonly use tomato-based sauces
- Marinara sauce is made from tomatoes
- Mozzarella is a type of cheese commonly paired with pasta and tomatoes
- This combination matches the user's requirements

**Generating Structured Output:** The understanding system creates a structured representation:

```
{
  "primary_intent": "request_cooking_assistance",
  "secondary_intent": "specify_preferences",
  "task": {
    "type": "cooking",
    "dish": "pasta",
    "required_ingredients": ["tomatoes", "cheese"],
    "user_involvement": "collaborative"
  },
  "response_type": "suggest_recipe_and_confirm"
}
```

**Handling Ambiguity:** The system recognizes potential ambiguities and how it resolved them:

"Pasta" could mean the dried pasta itself or a complete pasta dish. Context suggests a complete dish since the user mentions additional ingredients.

"Something with tomatoes and cheese" is vague. The robot interprets this as a requirement that both ingredients be present, not an either/or choice. The use of "and" rather than "or" supports this interpretation.

The request could mean the user wants the robot to cook autonomously or provide guidance. The phrase "help me make" suggests collaborative cooking, so the robot offers a recipe suggestion rather than immediately starting to cook.

**Confidence Assessment:** The system evaluates its confidence in this interpretation. It is highly confident about:
- The intent to get cooking assistance
- The dish type (pasta)
- The required ingredients (tomatoes and cheese)

It is moderately confident about:
- The specific type of cheese (defaulting to mozzarella as common choice)
- The preferred level of robot involvement

Because of this moderate confidence on some aspects, the robot's response asks for confirmation: "Would you like me to suggest a recipe..." This allows the user to clarify if the robot's interpretation was incorrect.

### Why This Understanding Succeeded

Several factors contributed to successful natural language understanding:

The user provided clear context by explicitly stating both the main request and the constraints. The robot's knowledge base included relevant information about cooking and typical ingredient combinations. The system correctly identified the collaborative nature of the request rather than treating it as a command for autonomous action.

The structured approach to understanding, from tokens through to semantic analysis and intent recognition, allowed the robot to build a complete picture of what the user wanted. Finally, the robot's willingness to confirm its interpretation before acting ensured any misunderstandings could be corrected.

## Practical Notes

### Choosing Natural Language Understanding Systems

Several options exist for implementing natural language understanding in robots.

**Commercial NLU Services** like Google Dialogflow, Amazon Lex, Microsoft LUIS, and IBM Watson Assistant provide cloud-based natural language understanding. These services handle intent recognition, entity extraction, and dialogue management. They offer user-friendly interfaces for defining intents and training models.

These services work well for conversational robots with standard dialogue patterns. They require internet connectivity and involve usage costs but save significant development time.

**Open Source Frameworks** like Rasa, spaCy, and Stanford CoreNLP provide natural language understanding capabilities that run on your own servers or robot hardware. These offer more control and privacy but require more technical expertise.

Rasa is particularly popular for building conversational AI systems. It provides tools for intent classification, entity extraction, and dialogue management with good documentation.

spaCy excels at core NLU tasks like tokenization, part-of-speech tagging, and named entity recognition. It is fast and efficient, suitable for real-time robot applications.

**Large Language Model APIs** like OpenAI's GPT models, Anthropic's Claude, or open alternatives can perform natural language understanding as part of their general capabilities. These models understand context and nuance without extensive training on specific domains.

These are powerful but require careful prompting to extract structured information reliably. They work well for open-ended understanding but may need additional processing for robot control tasks.

**Hybrid Approaches** combine multiple tools. You might use spaCy for basic text processing, a custom classifier for intent recognition, and a large language model for handling unusual or complex requests.

### Defining Intents and Entities

For many robot applications, you will define specific intents and entities the robot should recognize.

**Intent Definition** involves identifying what actions or information types users might request. For a home assistant robot, intents might include:
- turn_on_light
- turn_off_light
- set_temperature
- get_weather
- set_alarm
- answer_question

Each intent represents a distinct action the robot can take. Start with common requests and expand based on actual usage.

**Entity Definition** specifies what information pieces the robot should extract. For the intents above, entities might include:
- device_name (living room light, bedroom fan)
- temperature_value (72 degrees, 21 celsius)
- location (Paris, New York)
- time (7 AM, tomorrow, in 5 minutes)

Define entities based on what information the robot needs to fulfill each intent.

**Training Examples** help the system learn to recognize intents and entities. Provide multiple ways of expressing each intent:

For "turn_on_light":
- "Turn on the kitchen light"
- "Switch on the light in the kitchen"
- "Can you turn the kitchen light on?"
- "Kitchen light on please"
- "I need the kitchen light"

Include variations in word order, politeness level, and phrasing. More diverse examples lead to better recognition.

### Handling Domain-Specific Language

Robots operating in specialized fields need to understand domain-specific terminology.

**Medical Robots** must recognize medical terms, medication names, and symptom descriptions. Standard natural language understanding models may not know specialized vocabulary.

**Industrial Robots** need to understand manufacturing terms, part numbers, and process-specific language.

**Custom Vocabulary Training** involves adding domain-specific words and phrases to your natural language understanding system. Most systems allow extending their vocabulary with custom terms.

**Domain Adaptation** fine-tunes language models on text from your specific domain. This helps the system learn typical phrasing and terminology in that field.

### Measuring Understanding Quality

Evaluate your natural language understanding system's performance using several metrics.

**Intent Recognition Accuracy** measures how often the system correctly identifies the user's intent. Test with diverse examples including edge cases.

**Entity Extraction Precision** measures how many extracted entities are correct. High precision means the system rarely identifies entities incorrectly.

**Entity Extraction Recall** measures how many entities the system successfully finds. High recall means the system rarely misses important information.

**End-to-End Task Success** measures whether the robot ultimately does what the user wanted. This is the most important metric because it reflects actual usefulness.

Test with real users speaking naturally, not just scripted examples. Record which requests succeed and which fail to identify improvement areas.

### Handling Errors and Uncertainty

Natural language understanding systems sometimes make mistakes or face ambiguous input. Design your system to handle these gracefully.

**Confidence Scoring** tells you how certain the system is about its interpretation. Most natural language understanding systems provide confidence scores with their outputs.

Set confidence thresholds. For high-confidence interpretations, proceed with the action. For medium confidence, confirm with the user. For low confidence, ask for clarification.

**Clarification Dialogues** help resolve ambiguity. When uncertain, ask specific questions:
- "Did you want the red box or the blue box?"
- "Should I turn on all lights or just the kitchen light?"
- "Do you mean tomorrow morning or tomorrow evening?"

Make questions clear and limit choices to avoid overwhelming users.

**Fallback Strategies** activate when understanding completely fails. The robot might:
- Ask the user to rephrase their request
- Suggest common actions the user might want
- Transfer to a human operator for complex queries
- Provide help information about what the robot can do

Never pretend to understand when you don't. Honest acknowledgment of limitations builds user trust.

### Context Management

Maintaining context across conversation turns is essential for natural dialogue.

**Conversation State Tracking** stores information about what has been discussed. This includes:
- Entities mentioned in previous turns
- The current topic or task
- User preferences expressed earlier
- Incomplete information being gathered

Implement a context manager that updates as the conversation progresses.

**Anaphora Resolution** handles pronouns and references like "it," "that," "the red one." The system must determine what these refer to based on recent context.

Store recently mentioned entities and use recency and salience to resolve references. The most recently mentioned entity matching the reference type is often the correct one.

**Context Reset** recognizes when conversations shift to new topics. If someone says "Never mind, let me ask something else," clear the previous context.

### Integration with Robot Systems

Natural language understanding produces structured output that other robot systems must use.

**Action Planning** takes intent and entities from natural language understanding and determines what physical or computational actions the robot should perform. This might involve navigation, manipulation, information retrieval, or other capabilities.

**Knowledge Base Integration** connects natural language understanding to the robot's knowledge about the world, its environment, and its capabilities. When someone asks "Where is the meeting room?" the robot queries its internal map.

**Sensor Fusion** combines language understanding with perceptual information. If someone says "Bring me that cup," the robot must use vision to identify which cup is "that cup" based on context and pointing gestures.

**Multi-Modal Understanding** integrates language with other input modalities like gestures, gaze direction, or touchscreen inputs. Someone might point while saying "Go there" - the robot must combine both signals.

### Privacy and Ethics

Natural language understanding involves processing potentially sensitive information.

**Data Minimization** means only extracting and storing information necessary for the robot's function. Don't log or retain unnecessary details about what users say.

**Sensitive Information Handling** requires special care for personal data, medical information, financial details, or other private content. Encrypt stored data and limit access.

**Informed Consent** ensures users know their speech is being processed and understand how the robot uses language understanding. Provide clear privacy policies.

**Bias Awareness** recognizes that natural language understanding systems can reflect biases in their training data. Systems might perform worse for some dialects, accents, or cultural communication styles. Test with diverse users and work to ensure fair performance.

**Misuse Prevention** considers how natural language understanding could be exploited. Robots should refuse requests that are illegal, harmful, or violate ethics guidelines. Implement appropriate content filtering.

### Continuous Improvement

Natural language understanding systems improve over time with proper feedback mechanisms.

**User Corrections** provide valuable training data. When users correct misunderstandings, log these corrections and use them to improve the system.

**Usage Analytics** track which intents are common, where errors occur, and what types of requests cause problems. This guides development priorities.

**Active Learning** identifies cases where the system is uncertain and asks users to provide correct labels. This efficiently gathers training data for difficult cases.

**Regular Retraining** incorporates new data and examples. As the robot encounters more diverse language, retrain models to handle this variation better.

## Summary

Natural language understanding transforms text into meaning that robots can act upon. After speech recognition converts speech to text, natural language understanding extracts intent, identifies important information, resolves ambiguities, and produces structured representations of what people want.

The process involves multiple stages: tokenization breaks text into pieces, part-of-speech tagging identifies grammatical roles, named entity recognition finds important references, parsing analyzes sentence structure, semantic analysis interprets meaning, intent recognition determines what action is requested, and slot filling extracts specific details needed to complete the task.

Natural language understanding faces challenges from ambiguity, context dependence, and the many ways people express ideas. Modern systems use machine learning, especially deep learning and large language models, to handle this complexity. These systems learn patterns from vast amounts of example text.

Understanding natural language is critical for robots that interact with humans. It enables humanoid assistants to comprehend nuanced communication, allows service robots to handle diverse requests, and makes robots accessible to all users regardless of their technical expertise.

Implementing natural language understanding requires choosing appropriate tools, defining intents and entities for your domain, training with diverse examples, handling errors gracefully, maintaining conversational context, and integrating with other robot systems.

Privacy protection, bias awareness, and ethical considerations are essential when processing human language. Systems should minimize data collection, handle sensitive information carefully, and ensure fair performance across diverse users.

As natural language understanding technology continues advancing, robots will better grasp what people mean, not just what they say. This will enable more natural, helpful, and collaborative human-robot interaction.